#!/usr/bin/env python3
"""Non-ROS runtime entry point for camera, perception, motion and monitoring."""
import os
from argparse import ArgumentParser
from ctypes import c_bool
from multiprocessing import Process, Queue, Value

from src.scenes import Manual, scene_initiator
from src.utils import getkey, log, CameraBroadcaster, CAMERA_INFO, Controller
from src.actions import Stop
from src.utils.capture_preview import should_overlay_results, should_start_monitor
from src.utils.monitor_server import run_monitor_server
from src.utils.motion_gate import set_motion_enabled
from src.utils.pedestrian_gate import clear_pedestrian_resume, set_pedestrian_blocked
from src.utils.decision_gate import is_decision_enabled, set_decision_enabled
from src.utils.monitoring import publish_system, publish_event
from src.utils.performance import configure_profile, run_system_profiler
import time

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--mode', type=str, required=False, default='manual',
                        choices=['cmd', 'voice', 'manual','easy'])
    parser.add_argument('--monitor', action='store_true', help='start browser monitor service')
    parser.add_argument('--monitor-host', type=str, default='127.0.0.1')
    parser.add_argument('--monitor-port', type=int, default=8080)
    parser.add_argument('--monitor-telemetry-port', type=int, default=0,
                        help='UDP port for in-memory monitor telemetry; 0 means monitor-port + 1')
    parser.add_argument('--auto-capture', action='store_true',
                        help='save camera frames periodically while manual mode is running')
    parser.add_argument('--capture-interval', type=float, default=0.5,
                        help='seconds between automatic captures in manual mode')
    parser.add_argument('--capture-dir', type=str, default='capture/manual_auto',
                        help='directory for automatic captures in manual mode')
    parser.add_argument('--no-capture-preview', action='store_true',
                        help='do not start browser preview automatically when --auto-capture is used')
    parser.add_argument('--preview-overlay', action='store_true',
                        help='draw model outputs on the browser preview even in manual mode')
    parser.add_argument('--start-motion-enabled', action='store_true',
                        help='allow easy mode to move immediately without waiting for g')
    parser.add_argument('--start-decision-enabled', action='store_true',
                        help='allow easy mode to make decisions immediately without waiting for d')
    parser.add_argument('--profile', action='store_true',
                        help='record easy-mode performance metrics under logs/performance/')

    return parser.parse_args()


def build_camera_info(args):
    camera_info = dict(CAMERA_INFO)
    camera_info['auto_capture'] = {
        'enabled': args.auto_capture,
        'interval': args.capture_interval,
        'save_dir': args.capture_dir,
    }
    camera_info['overlay_results'] = should_overlay_results(args.mode, args.preview_overlay)
    return camera_info


def start_monitor(args, shared_memory_name, camera_info):
    capture_preview_enabled = not args.no_capture_preview
    monitor_enabled = should_start_monitor(args.mode, args.monitor, args.auto_capture, capture_preview_enabled)
    if not monitor_enabled:
        # Scene processes should not synchronously write monitor JSON on every frame.
        os.environ['OLD_CAR_DISABLE_STATE_PUBLISH'] = '1'
        os.environ.pop('OLD_CAR_MONITOR_TELEMETRY_PORT', None)
        return None, None

    os.environ.pop('OLD_CAR_DISABLE_STATE_PUBLISH', None)
    # Keep monitor updates responsive without making scene processes write JSON every frame.
    os.environ['OLD_CAR_MONITOR_STATE_INTERVAL'] = '0.1'
    telemetry_port = args.monitor_telemetry_port or args.monitor_port + 1
    os.environ['OLD_CAR_MONITOR_TELEMETRY_HOST'] = '127.0.0.1'
    os.environ['OLD_CAR_MONITOR_TELEMETRY_PORT'] = str(telemetry_port)

    stop_sign = Value(c_bool, False)
    process = Process(
        target=run_monitor_server,
        args=(shared_memory_name, camera_info),
        kwargs={
            'host': args.monitor_host,
            'port': args.monitor_port,
            'stop_sign': stop_sign,
            'telemetry_host': '127.0.0.1',
            'telemetry_port': telemetry_port,
            'initial_state': {
                'system': {'mode': args.mode, 'running': True},
                'camera': {
                    'status': 'running',
                    'width': camera_info.get('width', 0),
                    'height': camera_info.get('height', 0),
                    'fps': camera_info.get('fps', 0),
                    'memory_name': shared_memory_name or '',
                },
            },
        },
    )
    process.start()
    publish_system(mode=args.mode, running=True, camera_info=camera_info, memory_name=shared_memory_name)
    publish_event('monitor', f'Monitor started on {args.monitor_host}:{args.monitor_port}')
    if args.mode == 'manual' and args.auto_capture and capture_preview_enabled and not args.monitor:
        log.info('Capture preview is enabled. Open the monitor URL to view the camera.')
    return process, stop_sign


def stop_monitor(process, stop_sign):
    publish_system(running=False)
    if stop_sign is not None:
        stop_sign.value = True
    if process is not None:
        process.join(timeout=2)
        if process.is_alive():
            process.kill()
            process.join(timeout=1)


def start_system_profile(args, process_pids):
    if not args.profile:
        return None, None
    stop_sign = Value(c_bool, False)
    process = Process(target=run_system_profiler, args=(stop_sign, process_pids))
    process.start()
    return process, stop_sign


def stop_system_profile(process, stop_sign):
    if stop_sign is not None:
        stop_sign.value = True
    if process is not None:
        process.join(timeout=2)
        if process.is_alive():
            process.kill()
            process.join(timeout=1)


def initialize_gates(args):
    motion_enabled = True
    decision_enabled = True
    if args.mode == 'easy':
        motion_enabled = bool(args.start_motion_enabled)
        decision_enabled = bool(args.start_decision_enabled)
    set_motion_enabled(motion_enabled)
    set_decision_enabled(decision_enabled)
    set_pedestrian_blocked(False)
    clear_pedestrian_resume()
    publish_system(motion_enabled=motion_enabled, decision_enabled=decision_enabled)
    if args.mode == 'easy':
        if not decision_enabled:
            log.info('Decision gate is closed. Press d to start decision checks.')
        if not motion_enabled:
            log.info('Motion gate is closed. Press g to start the car.')


def set_motion_gate_enabled(enabled):
    set_motion_enabled(enabled)
    publish_system(motion_enabled=enabled)
    publish_event('motion_gate', f'Motion {"enabled" if enabled else "disabled"}')


def set_decision_gate_enabled(enabled):
    set_decision_enabled(enabled)
    publish_system(decision_enabled=enabled)
    publish_event('decision_gate', f'Decision {"enabled" if enabled else "disabled"}')


def start_car():
    set_decision_gate_enabled(True)
    set_motion_gate_enabled(True)


def toggle_decision_gate():
    enabled = not is_decision_enabled(default=False)
    set_decision_gate_enabled(enabled)
    return enabled


if __name__ == '__main__':
    args = parse_args()
    profile_dir = configure_profile(args.profile)
    if profile_dir is not None:
        log.info(f'Performance profiling enabled: {profile_dir}')
    camera_info = build_camera_info(args)
    log.info('start')
    initialize_gates(args)
    ctrl = Controller()
    msg_queue = Queue(maxsize=1)
    camera = CameraBroadcaster(camera_info)
    shared_memory_name = camera.memory_name
    camera_process = Process(target=camera.run)
    camera_process.start()
    monitor_process, monitor_stop_sign = start_monitor(args, shared_memory_name, camera_info)
    if args.mode == 'manual':
        task = Manual(shared_memory_name, camera_info, msg_queue)
        process = Process(target=task.loop)
        process.start()
        try:
            while True:
                key = getkey()
                if key == 'esc':
                    process.kill()
                    ctrl.execute(Stop())
                    camera.stop_sign.value = True
                    camera_process.join()
                    stop_monitor(monitor_process, monitor_stop_sign)
                    break
                else:
                    msg_queue.put(key)
        except (KeyboardInterrupt, SystemExit):
            process.kill()
            ctrl.execute(Stop())
            camera.stop_sign.value = True
            camera_process.join()
            stop_monitor(monitor_process, monitor_stop_sign)
            os.system('stty sane')
            log.info('stopping.')
    elif args.mode == 'cmd':
        process_list = []
        record_map = {}
        try:
            log.info(f'start reading cmd')
            while True:
                command = input().strip()
                if command == 'stop':
                    for p in process_list:
                        p.kill()
                    log.info(f'start put stop sign')
                    ctrl.execute(Stop())
                    camera.stop_sign.value = True
                    camera_process.join()
                    stop_monitor(monitor_process, monitor_stop_sign)
                    break
                elif command == 'clear':
                    for p in process_list:
                        p.kill()
                    process_list.clear()
                    ctrl.execute(Stop())
                    log.info(f'clear succ')
                    continue
                elif command == 'Manual':
                    log.error(f'Does not support switching from cmd mode to manual mode')
                    continue
                log.info(f'building scene {command}')
                scene = scene_initiator(command)
                log.info(f'{scene}')
                if scene is not None:
                    scene_obj = scene(shared_memory_name, camera_info, msg_queue)
                    process = Process(target=scene_obj.loop)
                    process.start()
                    process_list.append(process)

        except (KeyboardInterrupt, SystemExit):
            camera.stop_sign.value = True
            camera_process.join()
            stop_monitor(monitor_process, monitor_stop_sign)
            for process in process_list:
                process.kill()
            log.info('stopping.')

    elif args.mode == 'voice':
        stop_monitor(monitor_process, monitor_stop_sign)
        raise NotImplementedError('voice control is not currently supported.')
    elif args.mode == 'easy':
        process_list = []
        task2 = scene_initiator('LF_Lanenet')(shared_memory_name, camera_info, msg_queue)
        process_list.append(Process(target=task2.loop))
        task1 = scene_initiator('Helper')(shared_memory_name, camera_info, msg_queue)
        process_list.append(Process(target=task1.loop))
 

        for process in process_list:
            process.start()
        profile_process, profile_stop_sign = start_system_profile(
            args,
            {
                'camera': camera_process.pid,
                'lanenet': process_list[0].pid,
                'yolo': process_list[1].pid,
            },
        )
        try:
            while True:
                key = getkey()
                if key == 'esc':
                    for process in process_list:
                        process.kill()
                    ctrl.execute(Stop())
                    camera.stop_sign.value = True
                    camera_process.join()
                    stop_system_profile(profile_process, profile_stop_sign)
                    stop_monitor(monitor_process, monitor_stop_sign)
                    break
                elif key == 'd':
                    enabled = toggle_decision_gate()
                    log.info(f'Decision gate {"opened" if enabled else "closed"}.')
                elif key == 'g':
                    start_car()
                    log.info('Car started: decision and motion gates opened.')
                elif key == 'space':
                    set_motion_gate_enabled(False)
                    ctrl.execute(Stop())
                    log.info('Motion gate closed, car stopped.')
                else:
                    msg_queue.put(key)
        except (KeyboardInterrupt, SystemExit):
            camera.stop_sign.value = True
            camera_process.join()
            stop_system_profile(profile_process, profile_stop_sign)
            stop_monitor(monitor_process, monitor_stop_sign)
            os.system('stty sane')
            log.info('stopping.')
