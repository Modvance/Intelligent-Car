# FILE: src/scenes/lf.py (REVISED)
# -*- coding: utf-8 -*-
import os
import time
import cv2
import numpy as np
import math
from src.actions import SetServo, Start, TurnLeft, TurnRight, Advance, Stop, Sleep, SpinClockwise, SpinAntiClockwise
from src.scenes.base_scene import BaseScene
from src.utils import log
from src.utils.motion_gate import is_motion_enabled
from src.utils.monitoring import publish_scene
from src.models.quickLF import LFModel 

class LF_Lanenet(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue): 
            super().__init__(memory_name, camera_info, msg_queue)
            self.height = camera_info['height']
            self.width = camera_info['width']
            
            self.vis_window_name = "Lane Keeping View (Cropped)"
            self.enable_gui = camera_info.get('enable_gui', False)
            if self.enable_gui:
                cv2.namedWindow(self.vis_window_name, cv2.WINDOW_NORMAL)
                display_width, display_height = 1024, int(1024 * (self.height * 2 / 3) / self.width)
                cv2.resizeWindow(self.vis_window_name, display_width, display_height)

    def init_state(self):
        publish_scene('LF_Lanenet', status='loading_model')
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        model_path = os.path.join(project_root, 'weights', 'lanenet.om')
        if not os.path.exists(model_path):
            log.error(f'Cannot find lane model: {model_path}')
            publish_scene('LF_Lanenet', status='model_missing', data={'lane': {'model_path': model_path}})
            return True
        log.info(f'start init --- {self.__class__.__name__}')
        self.model = LFModel(model_path)
        log.info(f'{self.__class__.__name__} --- model init succ.')
        self.ctrl.execute(SetServo(servo=[90, 65]))
        publish_scene('LF_Lanenet', status='ready', data={'lane': {'model_path': model_path}})
        return False
    
    # --- HELPER METHODS ---
    
    def _transform_model_to_cropped_coords(self, lane_results_model, model_dims, cropped_dims):
        """
        Transforms lane parameters (k, b) and passes through the confidence score.
        Input: list of (k_model, b_model, confidence)
        Output: list of (k_new, b_new, confidence)
        """
        model_w, model_h = model_dims
        cropped_w, cropped_h = cropped_dims
        scale_x, scale_y = cropped_w / model_w, cropped_h / model_h
        
        lanes_in_cropped_coords = []
        if scale_y > 1e-6:
            for k_model, b_model, confidence in lane_results_model:
                k_new = k_model * (scale_x / scale_y)
                b_new = b_model * scale_x
                lanes_in_cropped_coords.append((k_new, b_new, confidence))
        return lanes_in_cropped_coords

    def _compute_steering_command(self, lane_params, image_width, image_height):
        """
        Compute steering command based on top-2 lane lines (k, b, confidence).
        Uses angle deviation from vertical direction as control signal.
        Returns a float: negative = turn left, positive = turn right, near zero = straight.
        """
        if not lane_params:
            return 0.0

        # Sort lanes by confidence and take the top 2
        sorted_lanes = sorted(lane_params, key=lambda x: x[2], reverse=True)
        line_fits = sorted_lanes[:2]

        left_line = None
        right_line = None

        for k, b, _ in line_fits:
            if k > 0:
                left_line = (k, b)
            elif k < 0:
                right_line = (k, b)

        def compute_intersection(line1, line2):
            k1, b1 = line1
            k2, b2 = line2
            if k1 == k2:
                return None
            y = (b2 - b1) / (k1 - k2)
            x = k1 * y + b1
            return (x, y)

        bottom_y = image_height
        
        # Case 1: Both lanes exist
        if left_line and right_line:
            k1, b1 = left_line
            k2, b2 = right_line
            # x1 = k1 * bottom_y + b1
            # x2 = k2 * bottom_y + b2

            intersection = compute_intersection(left_line, right_line)
            if intersection is None:
                return 0.0  # parallel fallback

            inter_x, inter_y = intersection
            # base_x = (x1 + x2) / 2
            base_x = image_width / 2

            dx = inter_x - base_x
            dy = inter_y - bottom_y
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                return 0.0

            cos_theta = (-dy) / norm
            angle_rad = math.acos(max(min(cos_theta, 1), -1))
            if dx < 0:
                angle_rad = -angle_rad

            return angle_rad  # final output: angle-based steering
            
        elif len(line_fits) == 1:
            k, b, _ = line_fits[0]

            def angle_with_vertical(k):
                cos_theta = 1 / math.sqrt(k ** 2 + 1)
                angle_rad = math.acos(cos_theta)
                return angle_rad if k < 0 else -angle_rad

            return angle_with_vertical(k)

        # Case 3: No valid lanes
        return 0.0

    def _draw_visualization(self, image, lane_params, steering_command):
        """Input is a list of (k, b, confidence) tuples."""
        for k, b, confidence in lane_params:
            y1, y2 = 0, image.shape[0] - 1
            x1, x2 = int(k * y1 + b), int(k * y2 + b)
            retval, pt1, pt2 = cv2.clipLine((0, 0, image.shape[1], image.shape[0]), (x1, y1), (x2, y2))
            if retval:
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)
                # Display confidence score on the line
                text_pos = ( (pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2 )
                cv2.putText(image, str(confidence), text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        h, w, _ = image.shape
        arrow_start_pt = (w // 2, h - 20)
        angle_rad = steering_command
        arrow_end_x = int(arrow_start_pt[0] + 100 * np.sin(angle_rad))
        arrow_end_y = int(arrow_start_pt[1] - 100 * np.cos(angle_rad))
        cv2.arrowedLine(image, arrow_start_pt, (arrow_end_x, arrow_end_y), (0, 0, 255), 7, line_type=cv2.LINE_AA, tipLength=0.4)

    def loop(self):
        ret = self.init_state()
        if ret: log.error(f'{self.__class__.__name__} init failed.'); return

        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
        log.info(f'LF loop start - Confidence Filtering Enabled')
        publish_scene('LF_Lanenet', status='running')
        
        # --- TUNING PARAMETER ---
        # The minimum number of pixels a lane must have to be considered valid.
        LANE_CONFIDENCE_THRESHOLD = 1000
        prev_steering = None
        motion_started = False
        
        try:
            while True:
                if self.stop_sign.value: break
                if self.pause_sign.value: continue
                log.info("--------------------------- Lanenet process ---------------------------------")
                # 1. CROP IMAGE
                img_cropped = frame[:, :]
                cropped_h, cropped_w, _ = img_cropped.shape
                
                # 2. GET PREDICTIONS: Returns (k, b, confidence) tuples
                lane_results_model, inference_time = self.model.pred(img_cropped)
                
                lane_results_model = sorted(lane_results_model, key=lambda x: x[2], reverse=True)  # Sort by confidence
                # 3. CONFIDENCE FILTERING: Keep only high-confidence lanes
                # confident_lanes_model = [lane for lane in lane_results_model if lane[2] > LANE_CONFIDENCE_THRESHOLD]
                confident_lanes_model = lane_results_model[:2]
                
                log.info(f"Detected {len(lane_results_model)} raw lanes, kept {len(confident_lanes_model)} after confidence filtering.")

                # 4. TRANSFORM COORDS of confident lanes
                lanes_in_cropped_coords = self._transform_model_to_cropped_coords(
                    confident_lanes_model, 
                    (self.model.model_width, self.model.model_height), 
                    (cropped_w, cropped_h)
                )

                # 5. DECISION MAKING based on confident lanes
                steering_command = self._compute_steering_command(lanes_in_cropped_coords, cropped_w, cropped_h)
                '''
                # 6. VISUALIZATION of confident lanes
                vis_frame = img_cropped.copy()
                self._draw_visualization(vis_frame, lanes_in_cropped_coords, steering_command)
                cv2.imshow(self.vis_window_name, vis_frame)
                cv2.waitKey(1)
                '''
                log.info(f'Steering Command = {steering_command:.2f}')
                
                alpha = 0.8
                if prev_steering is None:
                  prev_steering = steering_command
                filtered_steering = alpha * steering_command + (1 - alpha) * prev_steering
                prev_steering = steering_command
                publish_scene(
                    'LF_Lanenet',
                    status='running',
                    data={
                        'lane': {
                            'raw_count': len(lane_results_model),
                            'kept_count': len(confident_lanes_model),
                            'lanes': [
                                {
                                    'k': float(k),
                                    'b': float(b),
                                    'confidence': float(confidence),
                                }
                                for k, b, confidence in lanes_in_cropped_coords
                            ],
                            'steering_command': float(steering_command),
                            'filtered_steering': float(filtered_steering),
                            'inference_time': float(inference_time),
                        }
                    },
                )

                # 7. EXECUTE CONTROL
                if not is_motion_enabled(default=True):
                    motion_started = False
                    log.debug('Motion gate closed, LF_Lanenet skips movement command.')
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue
                if not motion_started:
                    self.ctrl.execute(Start())
                    motion_started = True

                LEFT_MAX_DEGREE, RIGHT_MAX_DEGREE = -15.0, 15.0
                LEFT_TURN_THRESHOLD = math.radians(LEFT_MAX_DEGREE)
                RIGHT_TURN_THRESHOLD = math.radians(RIGHT_MAX_DEGREE)
                
                
                if LEFT_TURN_THRESHOLD <= filtered_steering <= RIGHT_TURN_THRESHOLD:
                    self.ctrl.execute(Advance(speed=26))
                    #self.ctrl.execute(Sleep(0.3))
                elif filtered_steering > RIGHT_TURN_THRESHOLD:
                    # self.ctrl.execute(SpinClockwise(speed=30))
                    self.ctrl.execute(TurnRight(speed=15, degree=filtered_steering))
                    sleep_time = filtered_steering
                    #self.ctrl.execute(Sleep(sleep_time))
                else:
                    # self.ctrl.execute(SpinAntiClockwise(speed=30))
                    self.ctrl.execute(TurnLeft(speed=15, degree=-filtered_steering))
                    sleep_time = - filtered_steering
                    #self.ctrl.execute(Sleep(sleep_time))
                self.ctrl.execute(Sleep(0.1))
                # self.ctrl.execute(Stop())
                '''
                
                if LEFT_TURN_THRESHOLD <= filtered_steering <= RIGHT_TURN_THRESHOLD:
                    self.ctrl.execute(Advance(speed=25))
                    #self.ctrl.execute(Sleep(0.3))
                elif filtered_steering > RIGHT_TURN_THRESHOLD:
                    # self.ctrl.execute(SpinClockwise(speed=30))
                    if filtered_steering > 45:
                        self.ctrl.execute(TurnRight(speed=10, degree=filtered_steering*2))
                    elif filtered_steering > 30:
                        self.ctrl.execute(TurnRight(speed=15, degree=filtered_steering*1.2))
                    else:
                        self.ctrl.execute(TurnRight(speed=20, degree=filtered_steering*0.8))
                    sleep_time = filtered_steering
                    #self.ctrl.execute(Sleep(sleep_time))
                elif filtered_steering < LEFT_TURN_THRESHOLD:
                    # self.ctrl.execute(SpinAntiClockwise(speed=30))
                    if filtered_steering < -45:
                        self.ctrl.execute(TurnLeft(speed=14, degree=-filtered_steering*2))
                    elif filtered_steering < -30:
                        self.ctrl.execute(TurnLeft(speed=18, degree=-filtered_steering*1.2))
                    else:
                        self.ctrl.execute(TurnLeft(speed=22, degree=-filtered_steering*0.8))
                    sleep_time = - filtered_steering
                    #self.ctrl.execute(Sleep(sleep_time))
                self.ctrl.execute(Sleep(0.1))
                # self.ctrl.execute(Stop())
                '''
                    
                # 8. Get next frame
                frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)

        except Exception as e:
            log.error(f'LF loop error: {e}')
        finally:
            self.ctrl.execute(Stop())
            if self.enable_gui:
                cv2.destroyAllWindows()
            publish_scene('LF_Lanenet', status='stopped')
            log.info("LF loop finished.")
