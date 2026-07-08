#!/usr/bin/env python3
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
from src.utils.monitoring import publish_system, publish_event
import time

def parse_args():
    parser = ArgumentParser()
    parser.add_argument('--mode', type=str, required=False, default='manual',
                        choices=['cmd', 'voice', 'manual','easy'])
    parser.add_argument('--monitor', action='store_true', help='start browser monitor service')
    parser.add_argument('--monitor-host', type=str, default='127.0.0.1')
    parser.add_argument('--monitor-port', type=int, default=8080)
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
    publish_system(mode=args.mode, running=True, camera_info=camera_info, memory_name=shared_memory_name)
    capture_preview_enabled = not args.no_capture_preview
    if not should_start_monitor(args.mode, args.monitor, args.auto_capture, capture_preview_enabled):
        return None, None

    stop_sign = Value(c_bool, False)
    process = Process(
        target=run_monitor_server,
        args=(shared_memory_name, camera_info),
        kwargs={'host': args.monitor_host, 'port': args.monitor_port, 'stop_sign': stop_sign},
    )
    process.start()
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


def initialize_motion_gate(args):
    enabled = True
    if args.mode == 'easy':
        enabled = bool(args.start_motion_enabled)
    set_motion_enabled(enabled)
    publish_system(motion_enabled=enabled)
    if args.mode == 'easy' and not enabled:
        log.info('Motion gate is closed. Press g to start moving.')


def set_motion_gate_enabled(enabled):
    set_motion_enabled(enabled)
    publish_system(motion_enabled=enabled)
    publish_event('motion_gate', f'Motion {"enabled" if enabled else "disabled"}')


if __name__ == '__main__':
    args = parse_args()
    camera_info = build_camera_info(args)
    log.info('start')
    initialize_motion_gate(args)
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
        try:
            while True:
                key = getkey()
                if key == 'esc':
                    for process in process_list:
                        process.kill()
                    ctrl.execute(Stop())
                    camera.stop_sign.value = True
                    camera_process.join()
                    stop_monitor(monitor_process, monitor_stop_sign)
                    break
                elif key == 'g':
                    set_motion_gate_enabled(True)
                    log.info('Motion gate opened.')
                elif key == 'space':
                    set_motion_gate_enabled(False)
                    ctrl.execute(Stop())
                    log.info('Motion gate closed, car stopped.')
                else:
                    msg_queue.put(key)
        except (KeyboardInterrupt, SystemExit):
            camera.stop_sign.value = True
            camera_process.join()
            stop_monitor(monitor_process, monitor_stop_sign)
            os.system('stty sane')
            log.info('stopping.')
