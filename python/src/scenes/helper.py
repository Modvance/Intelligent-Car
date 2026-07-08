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
    execute_stop_sign,
    execute_turnaround_entry,
    execute_turnaround_finish,
)
from src.models import YoloV5
from src.scenes.base_scene import BaseScene
from src.utils import log
from src.utils.motion_gate import is_motion_enabled
from src.utils.monitoring import publish_scene

class Helper(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.det = None
        self.cls = None
        self.last_cates = [] 

    def init_state(self):
        publish_scene('Helper', status='loading_model')
        log.info(f'start init {self.__class__.__name__}')
        det_path = os.path.join(os.getcwd(), 'weights', 'yolo.om')
        log.info(f'Weight file path: {det_path}')
        if not os.path.exists(det_path):
            log.error(f'Cannot find the offline inference model(.om) file needed for {self.__class__.__name__} scene.')
            publish_scene('Helper', status='model_missing', data={'detections': [], 'helper': {'model_path': det_path}})
            return True
        self.det = YoloV5(det_path)
        log.info(f'{self.__class__.__name__} model initialized successfully.')
        self.ctrl.execute(SetServo(servo=[90, 65]))
        log.info('Initial servo set: SetServo(servo=[90, 65])')
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
        count = 0
        park_flag = True
        back_flag = True
        try:
            while True:
                if self.stop_sign.value:
                    log.info('stop_sign detected, Helper process will exit.')
                    break
                if self.pause_sign.value:
                    log.info('pause_sign detected, Helper process will skip this inference.')
                    continue
                start = time.time()
                img_bgr = frame.copy()
                log.info('Frame acquired, ready for inference.')
                bboxes = self.det.infer(img_bgr)
                log.info(f'Inference result: {bboxes}')
                bboxes = sorted(bboxes, key=lambda x: x[5], reverse=True)
                cates = [bbox[4] for bbox in bboxes]
                detections = []
                for bbox in bboxes[:10]:
                    x1, y1, x2, y2, cate, score = bbox
                    detections.append({
                        'cate': cate,
                        'score': float(score),
                        'box': [int(x1), int(y1), int(x2), int(y2)],
                    })
                publish_scene(
                    'Helper',
                    status='running',
                    data={
                        'detections': detections,
                        'helper': {
                            'last_cates': list(self.last_cates),
                            'inference_time': float(time.time() - start),
                        },
                    },
                )
                if not is_motion_enabled(default=True):
                    log.debug('Motion gate closed, Helper skips sign action trigger.')
                    log.info(f'Inference time: {time.time() - start:.3f} seconds')
                    continue
                
                for x1, y1, x2, y2, cate, score in bboxes:
                    print(x1,y1,x2,y2,cate,score)
                    if cate == 'back':
                        continue
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
                        if cate == 'park' and park_flag and score >= 0.8:
                            if x>640 and y >= 50:
                                log.info('Execute park: park')
                                execute_park(self.ctrl, clear_func=self.clear)
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
                        if (cate == 'left' or cate == 'right') and count<3:
                            if 420 < x < 950 and y >= 250:
                                log.info('Execute left turn: TurnLeft')
                                execute_left_turn(self.ctrl, clear_func=self.clear)
                                count += 1
                                last_action = cate
                                break
                        if (cate == 'left' or cate == 'right') and count>=3:
                            if 420 < x < 950 and y >= 180:
                                log.info('Execute right turn: TurnRight')
                                execute_right_turn(self.ctrl, clear_func=self.clear)
                                last_action = cate
                                break
                        
                        if cate == 'sideway': #cate == 'back' or 
                            if 420 < x < 950 and y >= 250:
                                if back_flag:
                                    back_flag = False
                                    log.info('Execute turnaround: 1 TurnAround')
                                    execute_turnaround_entry(self.ctrl, clear_func=self.clear)
                                    last_action = cate
                                    break
                                else:
                                    log.info('Execute turnaround: 2 TurnAround')
                                    execute_turnaround_finish(self.ctrl, clear_func=self.clear)
                                    last_action = cate
                                    break
                        
                        if cate == 'stop':
                             if abs(x - 640) < 320 and abs(y - 320) < 160:
                                log.info('Execute stop: Stop')
                                execute_stop_sign(self.ctrl)
                                # self.clear() # 清空旧指令影响
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
            publish_scene('Helper', status='stopped')

