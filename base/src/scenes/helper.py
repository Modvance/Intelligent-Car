#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import cv2

import numpy as np
from src.actions import SetServo, Stop
from src.actions.sign_actions import (
    execute_left_turn,
    execute_park,
    execute_right_turn,
    execute_turnaround_entry,
)
from src.models import YoloV5
from src.scenes.base_scene import BaseScene
from src.utils import log
from src.utils.decision_gate import is_decision_enabled
from src.utils.motion_gate import is_motion_enabled, is_sign_action_active, request_motion_resume, set_sign_action_active
from src.utils.pedestrian_gate import (
    clear_pedestrian_resume,
    is_human_in_stop_region,
    is_pedestrian_blocked,
    request_pedestrian_resume,
    set_pedestrian_blocked,
    should_defer_sign_decisions,
    update_pedestrian_state,
)
from src.utils.monitoring import publish_decision, publish_scene
from src.utils.performance import PerformanceRecorder


TURNAROUND_MARKER = "back"
BACK_TRIGGER_Y = 300
BACK_REARM_Y = 240
MAX_TURNAROUNDS = 2
PARK_RIGHT_TURN_GATE_ENABLED = False
MIN_RIGHT_TURNS_BEFORE_PARK = 2
HUMAN_LABEL = 'human'
HUMAN_SCORE_THRESHOLD = 0.30
HUMAN_REGION_X_MIN = 350
HUMAN_REGION_X_MAX = 750
HUMAN_REGION_Y_MIN = 300
HUMAN_CLEAR_FRAMES = 2  


def update_back_turnaround_state(marker_y, armed, turnaround_count, in_trigger_region=True):
    """Arm each distant back marker once and trigger at most two turnarounds."""
    if turnaround_count >= MAX_TURNAROUNDS:
        return False, turnaround_count, False
    if not armed:
        if marker_y <= BACK_REARM_Y:
            return True, turnaround_count, False
        return False, turnaround_count, False
    if in_trigger_region and marker_y >= BACK_TRIGGER_Y:
        return False, turnaround_count, True
    return True, turnaround_count, False


def can_trigger_park(right_turn_count, gate_enabled=PARK_RIGHT_TURN_GATE_ENABLED):
    """Apply the optional two-right-turn gate before allowing parking."""
    return not gate_enabled or right_turn_count >= MIN_RIGHT_TURNS_BEFORE_PARK


class Helper(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.det = None
        self.cls = None
        self.last_cates = [] 
        self.performance = PerformanceRecorder('yolo')

    def init_state(self):
        publish_scene('Helper', status='loading_model')
        log.info(f'start init {self.__class__.__name__}')
        set_pedestrian_blocked(False)
        clear_pedestrian_resume()
        det_path = os.path.join(os.getcwd(), 'weights', 'yolo.om')
        log.info(f'Weight file path: {det_path}')
        if not os.path.exists(det_path):
            log.error(f'Cannot find the offline inference model(.om) file needed for {self.__class__.__name__} scene.')
            publish_scene('Helper', status='model_missing', data={'detections': [], 'helper': {'model_path': det_path}})
            return True
        self.det = YoloV5(det_path)
        set_sign_action_active(False)
        log.info(f'{self.__class__.__name__} model initialized successfully.')
        self.ctrl.execute(SetServo(servo=[93, 162]))
        log.info('Initial servo set: SetServo(servo=[93, 162])')
        publish_scene('Helper', status='ready', data={'helper': {'model_path': det_path}})
        return False
        
    def draw_bbox(self, bbox, img0, color, wt, names):
        det_result_str = ''
        for idx, class_id in enumerate(bbox[:, 5]):
            if float(bbox[idx][4] < float(0.05)):
                continue
            img0 = cv2.rectangle(img0, (int(bbox[idx][0]), int(bbox[idx][1])), (int(bbox[idx][2]), int(bbox[idx][3])),
                                 color, wt)
            img0 = cv2.putText(img0, str(idx) + ' ' + names[int(class_id)], (int(bbox[idx][0]), int(bbox[idx][1] + 16)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            img0 = cv2.putText(img0, '{:.4f}'.format(bbox[idx][4]), (int(bbox[idx][0]), int(bbox[idx][1] + 32)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            det_result_str += '{} {} {} {} {} {}\n'.format(
                names[bbox[idx][5]], str(bbox[idx][4]), bbox[idx][0], bbox[idx][1], bbox[idx][2], bbox[idx][3])
        return img0
    
    def clear(self):
        # ? 插入：清空旧指令影响
        self.ctrl.last_modify_time = time.time()  # 标记这一刻为控制起点
        self.ctrl._save()                         # 保存状态，其他进程将同步这个时间

    def publish_would_trigger(self, cate, score, center):
        publish_scene(
            'Helper',
            status='would_trigger',
            data={'helper': {'would_trigger': cate, 'score': float(score), 'center': [int(center[0]), int(center[1])]}}
        )
        publish_decision(
            'sign',
            'Helper',
            status='would_trigger',
            action=cate,
            data={
                'score': float(score),
                'center': [int(center[0]), int(center[1])],
                'decision_enabled': True,
                'motion_enabled': False,
            },
        )

    def run_sign_action(self, action_name, score, center, action_func, clear_func=False, frame_to_decision_ms=None):
        """Run one landmark action using the original synchronous implementation."""
        set_sign_action_active(True)
        publish_decision(
            'sign',
            'Helper',
            status='triggered',
            action=action_name,
            data={
                'score': float(score),
                'center': [int(center[0]), int(center[1])],
                'decision_enabled': True,
                'motion_enabled': True,
            },
        )
        self.performance.event(
            'trigger',
            action=action_name,
            x=int(center[0]),
            y=int(center[1]),
            frame_to_decision_ms=frame_to_decision_ms,
        )
        try:
            if clear_func:
                action_func(self.ctrl, clear_func=self.clear)
            else:
                action_func(self.ctrl)
        except Exception:
            raise
        finally:
            set_sign_action_active(False)
    
    def loop(self):
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} initialization failed.')
            return
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
        publish_scene('Helper', status='running')
        last_action = None
        labels = self.det.names
        turn_count = 0
        right_turn_count = 0
        park_flag = True
        back_marker_armed = True
        turnaround_count = 0
        pedestrian_clear_frames = 0
        pedestrian_blocked = is_pedestrian_blocked(default=False)
        try:
            while True:
                if self.stop_sign.value:
                    log.info('stop_sign detected, Helper process will exit.')
                    break
                if self.pause_sign.value:
                    log.info('pause_sign detected, Helper process will skip this inference.')
                    continue
                start = time.time()
                profile_start = time.perf_counter()
                img_bgr = frame.copy()
                frame_copy_ms = (time.perf_counter() - profile_start) * 1000
                log.info('Frame acquired, ready for inference.')
                bboxes = self.det.infer(img_bgr)
                model_profile = getattr(self.det, 'last_profile', {})
                frame_to_result_ms = (time.perf_counter() - profile_start) * 1000
                self.performance.observe(
                    frame_copy_ms=frame_copy_ms,
                    preprocess_ms=model_profile.get('preprocess_ms'),
                    model_ms=model_profile.get('model_ms'),
                    postprocess_ms=model_profile.get('postprocess_ms'),
                    model_total_ms=model_profile.get('total_ms'),
                    frame_to_result_ms=frame_to_result_ms,
                )
                log.info(f'Inference result: {bboxes}')
                bboxes = sorted(bboxes, key=lambda x: x[5], reverse=True)
                human_detected, human_bbox = is_human_in_stop_region(
                    bboxes,
                    label=HUMAN_LABEL,
                    score_threshold=HUMAN_SCORE_THRESHOLD,
                    x_min=HUMAN_REGION_X_MIN,
                    x_max=HUMAN_REGION_X_MAX,
                    y_min=HUMAN_REGION_Y_MIN,
                )
                pedestrian_blocked, pedestrian_clear_frames, entered, exited = update_pedestrian_state(
                    pedestrian_blocked,
                    pedestrian_clear_frames,
                    human_detected,
                    clear_threshold=HUMAN_CLEAR_FRAMES,
                )
                if entered:
                    set_pedestrian_blocked(True)
                    clear_pedestrian_resume()
                    log.warning('Human entered the stop region; interrupting motion.')
                    self.ctrl.execute(Stop())
                elif exited:
                    set_pedestrian_blocked(False)
                    request_pedestrian_resume()
                    log.info('Human left the stop region; LaneNet may resume motion.')

                cates = [bbox[4] for bbox in bboxes]
                detections = []
                for bbox in bboxes[:10]:
                    x1, y1, x2, y2, cate, score = bbox
                    detections.append({
                        'cate': cate,
                        'score': float(score),
                        'box': [int(x1), int(y1), int(x2), int(y2)],
                    })
                decision_enabled = is_decision_enabled(default=True)
                motion_enabled = is_motion_enabled(default=True)
                publish_scene(
                    'Helper',
                    status='running',
                    data={
                        'detections': detections,
                        'helper': {
                            'last_cates': list(self.last_cates),
                            'inference_time': float(time.time() - start),
                            'decision_enabled': bool(decision_enabled),
                            'motion_enabled': bool(motion_enabled),
                            'pedestrian_detected': bool(human_detected),
                            'pedestrian_blocked': bool(pedestrian_blocked),
                            'pedestrian_clear_frames': int(pedestrian_clear_frames),
                            'pedestrian_box': None if human_bbox is None else [int(v) for v in human_bbox[:4]],
                        },
                    },
                )
                publish_decision(
                    'sign',
                    'Helper',
                    status='running',
                    action='',
                    data={
                        'decision_enabled': bool(decision_enabled),
                        'motion_enabled': bool(motion_enabled),
                        'last_cates': list(self.last_cates),
                        'detections_count': len(detections),
                        'pedestrian_detected': bool(human_detected),
                        'pedestrian_blocked': bool(pedestrian_blocked),
                    },
                )
                if should_defer_sign_decisions(pedestrian_blocked):
                    log.info('Pedestrian stop gate is active; defer sign decisions for this frame.')
                    publish_scene(
                        'Helper',
                        status='paused_for_pedestrian',
                        data={'helper': {'pedestrian_blocked': True, 'pedestrian_box': None if human_bbox is None else [int(v) for v in human_bbox[:4]]}},
                    )
                    publish_decision(
                        'sign',
                        'Helper',
                        status='paused_for_pedestrian',
                        action='',
                        data={'decision_enabled': bool(decision_enabled), 'motion_enabled': bool(motion_enabled), 'pedestrian_blocked': True},
                    )
                    log.info(f'Inference time: {time.time() - start:.3f} seconds')
                    continue
                if not decision_enabled:
                    log.debug('Decision gate closed, Helper skips sign action checks.')
                    publish_scene('Helper', status='decision_locked', data={'helper': {'decision_enabled': False}})
                    publish_decision(
                        'sign',
                        'Helper',
                        status='decision_locked',
                        action='',
                        data={'decision_enabled': False, 'motion_enabled': bool(motion_enabled)},
                    )
                    log.info(f'Inference time: {time.time() - start:.3f} seconds')
                    continue

                if is_sign_action_active(default=False):
                    log.info('Sign action is active; Helper skips duplicate sign decisions.')
                    publish_scene(
                        'Helper',
                        status='sign_action_active',
                        data={'helper': {'sign_action_active': True}},
                    )
                    publish_decision(
                        'sign',
                        'Helper',
                        status='sign_action_active',
                        action='',
                        data={
                            'decision_enabled': bool(decision_enabled),
                            'motion_enabled': bool(motion_enabled),
                        },
                    )
                    log.info(f'Inference time: {time.time() - start:.3f} seconds')
                    continue
                
                for x1, y1, x2, y2, cate, score in bboxes:
                    print(x1,y1,x2,y2,cate,score)
                    log.info(f"Detected object: cate={cate}, score={score:.3f}, box=({x1},{y1},{x2},{y2})")
                    x, y = (x1 + x2) // 2, (y1 + y2) // 2
                    log.info(f"Object center: ({x}, {y}), size: ({x2-x1}, {y2-y1})")
                    log.info(f'det: {cate}')
                    '''
                    if last_action != cate and len(bboxes) > 1:
                        log.info(f'Different from last action and multiple objects detected, use last action: {last_action}')
                        cate = last_action
                    '''
                    if score < 0.30:
                        log.info(f'Score below threshold, skip this object: {score:.3f}')
                        continue
                    self.last_cates.append(cate)
                    if len(self.last_cates) > 5:
                        self.last_cates.pop(0)
                    log.info(f'Last 5 frame categories: {self.last_cates}')
                    if self.last_cates.count(cate) >= 1:
                        log.info(f'Multi-frame consistency passed, execute action: {cate}')
                        publish_scene(
                            'Helper',
                            status='triggered',
                            data={'helper': {'trigger': cate, 'score': float(score), 'center': [int(x), int(y)]}},
                        )
                        '''
                        if park_flag:
                            park_y = [(bbox[1]+bbox[3]) // 2 for bbox in bboxes if bbox[4]=='park']
                            if len(park_y)>=1 and park_y[0] >= 150:
                                
                                log.info('Execute park: park')
                                self.ctrl.execute(Advance(speed=30))
                                self.ctrl.execute(Sleep(1.0))
                                self.clear() # 清空旧指令影响
                                
                                self.ctrl.execute(ShiftRight(speed=40))
                                self.ctrl.execute(Sleep(0.5))
                                self.clear() # 清空旧指令影响
                                
                                self.ctrl.execute(Stop())
                                self.ctrl.execute(Sleep(2))
                                self.clear() # 清空旧指令影响
                                
                                self.ctrl.execute(ShiftLeft(speed=40))
                                self.ctrl.execute(Sleep(0.5))
                                self.clear() # 清空旧指令影响
                                
                                last_action = cate
                                break
                        '''
                        if cate == 'park' and park_flag and can_trigger_park(right_turn_count) and score >= 0.8:
                            if x>900 and y >= 230:
                                if not motion_enabled:
                                    log.info('Would execute park: park')
                                    self.publish_would_trigger(cate, score, (x, y))
                                    break
                                log.info('Execute park: park')
                                self.run_sign_action('park', score, (x, y), execute_park, clear_func=True, frame_to_decision_ms=frame_to_result_ms)
                                park_flag = False
                                
                                last_action = cate
                                break
                        '''
                        if cate == 'left':
                            if 420 < x < 950 and y >= 190:
                                log.info('Execute left turn: TurnLeft')
                                
                                self.ctrl.interrupt_and_execute(Advance(speed=28))
                                self.ctrl.execute(Sleep(0.5))
                                
                                # ? 插入：清空旧指令影响
                                self.ctrl.last_modify_time = time.time()  # 标记这一刻为控制起点
                                self.ctrl._save()                         # 保存状态，其他进程将同步这个时间
                                
                                self.ctrl.execute(SpinAntiClockwise(speed=32))
                                self.ctrl.execute(Sleep(0.5))
                                self.ctrl.execute(Stop())
                                self.ctrl.execute(Sleep(0.2))
                                
                                last_action = cate
                                break
                        if cate == 'right':
                            if 420 < x < 950 and y >= 210:
                                log.info('Execute right turn: TurnRight')
                                #self.ctrl.execute(TurnRight(speed=21, degree=1.1))
                                self.ctrl.interrupt_and_execute(Advance(speed=28))
                                self.ctrl.execute(Sleep(0.5))
                                
                                # ? 插入：清空旧指令影响
                                self.ctrl.last_modify_time = time.time()  # 标记这一刻为控制起点
                                self.ctrl._save()                         # 保存状态，其他进程将同步这个时间
                                
                                # 刚好90度的参数
                                self.ctrl.execute(SpinClockwise(speed=33))
                                self.ctrl.execute(Sleep(0.7))
                                self.ctrl.execute(Stop())
                                self.ctrl.execute(Sleep(0.2))
                                
                                last_action = cate
                                break
                        '''
                        if (cate == 'left' or cate == 'right') and turn_count < 3:
                            if 420 < x < 950 and y >= 270:
                                if not motion_enabled:
                                    log.info('Would execute left turn: TurnLeft')
                                    self.publish_would_trigger('left_turn', score, (x, y))
                                    break
                                log.info('Execute left turn: TurnLeft')
                                self.run_sign_action('left_turn', score, (x, y), execute_left_turn, clear_func=True, frame_to_decision_ms=frame_to_result_ms)
                                request_motion_resume()
                                turn_count += 1
                                last_action = cate
                                break
                        if (cate == 'left' or cate == 'right') and turn_count >= 3:
                            if 420 < x < 950 and y >= 270:
                                if not motion_enabled:
                                    log.info('Would execute right turn: TurnRight')
                                    self.publish_would_trigger('right_turn', score, (x, y))
                                    break
                                log.info('Execute right turn: TurnRight')
                                self.run_sign_action('right_turn', score, (x, y), execute_right_turn, clear_func=True, frame_to_decision_ms=frame_to_result_ms)
                                request_motion_resume()
                                right_turn_count += 1
                                last_action = cate
                                break
                        
                        if cate == TURNAROUND_MARKER:
                            was_armed = back_marker_armed
                            back_marker_armed, _, should_trigger = update_back_turnaround_state(
                                marker_y=y,
                                armed=back_marker_armed,
                                turnaround_count=turnaround_count,
                                in_trigger_region=420 < x < 950,
                            )
                            if not was_armed and back_marker_armed:
                                log.info('New back marker detected, turnaround rearmed.')
                            if should_trigger:
                                if not motion_enabled:
                                    log.info('Would execute turnaround: TurnAround')
                                    self.publish_would_trigger('turnaround_entry', score, (x, y))
                                    break
                                log.info('Execute turnaround: TurnAround')
                                self.run_sign_action('turnaround_entry', score, (x, y), execute_turnaround_entry, clear_func=True, frame_to_decision_ms=frame_to_result_ms)
                                request_motion_resume()
                                turnaround_count += 1
                                back_marker_armed = False
                                last_action = cate
                                break
                                
                        log.info('No matching action for detected category, do nothing.')
                    else:
                        log.info(f'Multi-frame consistency not passed, current category: {cate}, count: {self.last_cates.count(cate)}')
                '''
                # 转换格式
                bbox_array = []
                for bbox in bboxes:
                    x1, y1, x2, y2, cate, score = bbox
                    if float(score) < 0.05:
                        continue
                    class_id = labels.index(cate)  # 'park' -> 0
                    bbox_array.append([x1, y1, x2, y2, float(score), class_id])
                bbox_array = np.array(bbox_array)
                
                # draw image
                draw_img = frame.copy()
                draw_img = self.draw_bbox(bbox_array, draw_img, (0, 255, 0), 2, labels)
                
                # show image
                cv2.imshow(self.vis_window_name, draw_img)
                cv2.waitKey(1)
                '''
                
                log.info(f'Inference time: {time.time() - start:.3f} seconds')
        except KeyboardInterrupt:
            log.info('KeyboardInterrupt received, Helper process stopping, execute Stop')
            #cv2.destroyAllWindows()
            self.ctrl.execute(Stop())
            set_pedestrian_blocked(False)
            clear_pedestrian_resume()
            set_sign_action_active(False)
            publish_scene('Helper', status='stopped')

