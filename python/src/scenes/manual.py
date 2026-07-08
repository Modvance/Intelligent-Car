import datetime
import os
import time

import cv2
import numpy as np

from src.actions import Advance, Stop, SetServo, TurnLeft, TurnRight, SpinClockwise, SpinAntiClockwise, BackUp, LeftOblique, RightOblique,\
    ShiftLeft, ShiftRight, CustomAction
from src.actions.complex_actions import ComplexAction, TurnAround
from src.actions.sign_actions import get_manual_sign_action
from src.scenes.base_scene import BaseScene
from src.utils import log
from src.utils.image_capture import TimedImageCapture
from src.utils.monitoring import publish_scene


class Manual(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue):
        super().__init__(memory_name, camera_info, msg_queue)
        self.speed = 29
        self.save_dir = os.path.join(os.getcwd(), 'capture')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
        capture_info = camera_info.get('auto_capture', {})
        auto_save_dir = capture_info.get('save_dir') or os.path.join('capture', 'manual_auto')
        if not os.path.isabs(auto_save_dir):
            auto_save_dir = os.path.join(os.getcwd(), auto_save_dir)
        self.auto_capture = TimedImageCapture(
            auto_save_dir,
            interval=capture_info.get('interval', 0.5),
            enabled=capture_info.get('enabled', False),
            threaded=True,
        )

    def init_state(self):
        self.ctrl.execute(SetServo(servo=[90, 65]))

    def loop(self):
        ret = self.init_state()
        if ret:
            log.error(f'{self.__class__.__name__} init failed.')
            return
        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'{self.__class__.__name__} loop start')
        publish_scene('Manual', status='running', data={'manual': {
            'speed': self.speed,
            'last_key': '',
            'auto_capture_enabled': self.auto_capture.enabled,
            'capture_interval': self.auto_capture.interval,
            'capture_dir': str(self.auto_capture.save_dir),
        }, 'lane': None, 'detections': []})
        last_action = SetServo(servo=[90, 65])

        while True:
            try:
                capture_path = self.auto_capture.maybe_capture(frame)
                if capture_path:
                    log.info(f'auto capture saved: {capture_path}')
                if not self.msg_queue.empty():
                    key = self.msg_queue.get()
                else:
                    time.sleep(0.01)
                    continue
            except KeyboardInterrupt:
                self.ctrl.execute(Stop())
                break

            degree = 0
            sign_action = get_manual_sign_action(key)
            if sign_action is not None:
                action_name, action_func = sign_action
                log.info(f'execute manual sign action: {action_name}')
                action_func(self.ctrl)
                publish_scene('Manual', status='running', data={'manual': {
                    'speed': self.speed,
                    'last_key': key,
                    'last_action': action_name,
                    'auto_capture_enabled': self.auto_capture.enabled,
                    'capture_interval': self.auto_capture.interval,
                    'capture_dir': str(self.auto_capture.save_dir),
                }})
                continue
            if key == 'up':
                self.speed = min(self.speed + 1, 60)
            elif key == 'down':
                self.speed = max(self.speed - 1, 25)
            elif key == 'left':
                last_action = ShiftLeft()
            elif key == 'right':
                last_action = ShiftRight()
            elif key == 'w':
                last_action = Advance()
            elif key == 'a':
                last_action = TurnLeft(degree = 0.1)
            elif key == 's':
                last_action = BackUp()
            elif key == 'd':
                last_action = TurnRight(degree = 0.1)
            elif key == 'q':
                last_action = SpinAntiClockwise()
            elif key == 'e':
                last_action = SpinClockwise()
            elif key == 'l':
                last_action = ShiftRight()
            elif key == 'j':
                last_action = ShiftLeft()
            elif key == 'u':
                last_action = LeftOblique()
            elif key == 'p':
                last_action = RightOblique()
            elif key == 'space':
                last_action = Stop()
            elif key == 'esc':
                self.ctrl.execute(Stop())
                break
            elif key == 'c':
                save_img = frame.copy()
                cv2.imwrite(os.path.join(self.save_dir, f'{datetime.datetime.now()}.jpg'), save_img)
                log.info(f'image saved.')
            elif key == 'v':
                enabled = self.auto_capture.toggle()
                log.info(f'auto capture {"enabled" if enabled else "disabled"}: {self.auto_capture.save_dir}')
                publish_scene('Manual', status='running', data={'manual': {
                    'speed': self.speed,
                    'last_key': key,
                    'last_action': last_action.__class__.__name__,
                    'auto_capture_enabled': enabled,
                    'capture_interval': self.auto_capture.interval,
                    'capture_dir': str(self.auto_capture.save_dir),
                }})
                continue
            elif key == 't':
                last_action = CustomAction(motor_setting=[-62, 50, 50, -50])
            elif key == 'r':
                last_action = CustomAction(motor_setting=[55, -50, -50, 50])
            elif key == 'z':
                last_action = TurnAround()
            elif key == 'x':
                from src.actions.complex_actions import Spin
                last_action = Spin()
            else:
                continue

            if not isinstance(last_action, ComplexAction) and not isinstance(last_action, CustomAction):
                last_action.update_speed = False
                last_action.speed_setting = last_action.generate_speed_setting(speed=self.speed, degree=degree)
                last_action.fix_speed()
            publish_scene(
                'Manual',
                status='running',
                data={'manual': {
                    'speed': self.speed,
                    'last_key': key,
                    'last_action': last_action.__class__.__name__,
                    'auto_capture_enabled': self.auto_capture.enabled,
                    'capture_interval': self.auto_capture.interval,
                    'capture_dir': str(self.auto_capture.save_dir),
                }},
            )
            self.ctrl.execute(last_action)
