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
from src.utils.decision_gate import is_decision_enabled
from src.utils.motion_gate import consume_motion_resume, is_motion_enabled, is_sign_action_active
from src.utils.pedestrian_gate import (
    consume_pedestrian_resume,
    is_pedestrian_blocked,
    should_resume_after_pedestrian,
)
from src.utils.monitoring import publish_decision, publish_scene
from src.utils.performance import PerformanceRecorder
from src.models.quickLF import LFModel 


# Lane-following tuning knobs. These four parameters are independent so they
# can be tuned on the vehicle without changing lane detection logic.
STEERING_FILTER_ALPHA = 0.8
TURN_TRIGGER_DEG = 15.0
MIN_TURN_STRENGTH = 0.25
TURN_GAIN = 1.5
MAX_TURN_STRENGTH = 1
STRAIGHT_SPEED = 26
TURN_SPEED = 20

# Lateral correction uses the center of the same two fitted lane lines that
# already feed the heading calculation. Positive steering means turn right.
LANE_LOOKAHEAD_RATIO = 0.75
LANE_CENTER_TARGET_X_RATIO = 0.50
LATERAL_DEADBAND_PX = 10.0
LATERAL_GAIN = 1.6


def compute_turn_control(
    steering_rad,
    turn_trigger_deg=TURN_TRIGGER_DEG,
    turn_gain=TURN_GAIN,
    min_turn_strength=MIN_TURN_STRENGTH,
    max_turn_strength=MAX_TURN_STRENGTH,
):
    """Return whether to turn and a bounded differential-drive strength."""
    steering_magnitude = abs(float(steering_rad))
    trigger_rad = math.radians(float(turn_trigger_deg))
    if steering_magnitude <= trigger_rad:
        return False, 0.0

    minimum = min(float(min_turn_strength), float(max_turn_strength))
    strength = minimum + (steering_magnitude - trigger_rad) * float(turn_gain)
    return True, min(strength, float(max_turn_strength))


def compute_lateral_steering(
    lane_params,
    image_width,
    image_height,
    lookahead_ratio=LANE_LOOKAHEAD_RATIO,
    target_x_ratio=LANE_CENTER_TARGET_X_RATIO,
    deadband_px=LATERAL_DEADBAND_PX,
    lateral_gain=LATERAL_GAIN,
):
    """Return lateral steering plus geometry values for tuning and monitoring."""
    sorted_lanes = sorted(lane_params, key=lambda x: x[2], reverse=True)
    left_line = None
    right_line = None
    for k, b, _ in sorted_lanes[:2]:
        if k > 0:
            left_line = (k, b)
        elif k < 0:
            right_line = (k, b)

    target_x = float(image_width) * float(target_x_ratio)
    if left_line is None or right_line is None:
        return 0.0, None, None, target_x

    lookahead_y = float(image_height) * float(lookahead_ratio)
    left_x = left_line[0] * lookahead_y + left_line[1]
    right_x = right_line[0] * lookahead_y + right_line[1]
    lane_center_x = (left_x + right_x) / 2.0
    lateral_error_px = lane_center_x - target_x
    if abs(lateral_error_px) <= float(deadband_px):
        return 0.0, lateral_error_px, lane_center_x, target_x

    lateral_steering = (lateral_error_px / float(image_width)) * float(lateral_gain)
    return lateral_steering, lateral_error_px, lane_center_x, target_x


def combine_steering(heading_steering, lateral_steering):
    """Combine independent heading and lateral correction terms."""
    return float(heading_steering) + float(lateral_steering)

class LF_Lanenet(BaseScene):
    def __init__(self, memory_name, camera_info, msg_queue): 
            super().__init__(memory_name, camera_info, msg_queue)
            self.height = camera_info['height']
            self.width = camera_info['width']
            self.performance = PerformanceRecorder('lanenet')
            
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
        self.ctrl.execute(SetServo(servo=[93, 162]))
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
                profile_start = time.perf_counter()
                log.info("--------------------------- Lanenet process ---------------------------------")
                # 1. CROP IMAGE
                img_cropped = frame[:, :]
                cropped_h, cropped_w, _ = img_cropped.shape
                
                # 2. GET PREDICTIONS: Returns (k, b, confidence) tuples
                lane_results_model, inference_time = self.model.pred(img_cropped)
                model_profile = getattr(self.model, 'last_profile', {})
                self.performance.observe(
                    model_ms=model_profile.get('model_ms', inference_time),
                    postprocess_ms=model_profile.get('postprocess_ms'),
                    model_total_ms=model_profile.get('total_ms'),
                    frame_to_result_ms=(time.perf_counter() - profile_start) * 1000,
                )
                
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
                heading_steering = self._compute_steering_command(lanes_in_cropped_coords, cropped_w, cropped_h)
                lateral_steering, lateral_error_px, lane_center_x, lane_target_x = compute_lateral_steering(
                    lanes_in_cropped_coords,
                    cropped_w,
                    cropped_h,
                )
                steering_command = combine_steering(heading_steering, lateral_steering)
                '''
                # 6. VISUALIZATION of confident lanes
                vis_frame = img_cropped.copy()
                self._draw_visualization(vis_frame, lanes_in_cropped_coords, steering_command)
                cv2.imshow(self.vis_window_name, vis_frame)
                cv2.waitKey(1)
                '''
                log.info(
                    'Steering Command = %.2f (heading=%.2f, lateral=%.2f, lateral_error=%s px)',
                    steering_command,
                    heading_steering,
                    lateral_steering,
                    'n/a' if lateral_error_px is None else f'{lateral_error_px:.1f}',
                )
                
                alpha = STEERING_FILTER_ALPHA
                if prev_steering is None:
                  prev_steering = steering_command
                filtered_steering = alpha * steering_command + (1 - alpha) * prev_steering
                prev_steering = steering_command
                should_turn, turn_strength = compute_turn_control(filtered_steering)
                if not should_turn:
                    intended_action = 'advance'
                elif filtered_steering > 0:
                    intended_action = 'turn_right'
                else:
                    intended_action = 'turn_left'

                decision_enabled = is_decision_enabled(default=True)
                motion_enabled = is_motion_enabled(default=True)
                pedestrian_blocked = is_pedestrian_blocked(default=False)
                lane_data = {
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
                    'heading_steering': float(heading_steering),
                    'lateral_steering': float(lateral_steering),
                    'lateral_error_px': None if lateral_error_px is None else float(lateral_error_px),
                    'lane_center_x': None if lane_center_x is None else float(lane_center_x),
                    'lane_target_x': float(lane_target_x),
                    'filtered_steering': float(filtered_steering),
                    'turn_strength': float(turn_strength),
                    'turn_trigger_deg': float(TURN_TRIGGER_DEG),
                    'min_turn_strength': float(MIN_TURN_STRENGTH),
                    'turn_gain': float(TURN_GAIN),
                    'lateral_deadband_px': float(LATERAL_DEADBAND_PX),
                    'lateral_gain': float(LATERAL_GAIN),
                    'intended_action': intended_action,
                    'decision_enabled': bool(decision_enabled),
                    'motion_enabled': bool(motion_enabled),
                    'pedestrian_blocked': bool(pedestrian_blocked),
                    'inference_time': float(inference_time),
                }
                publish_scene(
                    'LF_Lanenet',
                    status='running',
                    data={'lane': lane_data},
                )
                publish_decision(
                    'lane',
                    'LF_Lanenet',
                    status='running',
                    action=intended_action,
                    data=lane_data,
                )

                # 7. DECISION / EXECUTE CONTROL
                if not decision_enabled:
                    motion_started = False
                    log.debug('Decision gate closed, LF_Lanenet skips movement decision.')
                    publish_scene(
                        'LF_Lanenet',
                        status='decision_locked',
                        data={'lane': {'would_action': '', 'decision_enabled': False, 'motion_enabled': bool(motion_enabled)}},
                    )
                    publish_decision(
                        'lane',
                        'LF_Lanenet',
                        status='decision_locked',
                        action=intended_action,
                        data={'decision_enabled': False, 'motion_enabled': bool(motion_enabled)},
                    )
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue

                if not motion_enabled:
                    motion_started = False
                    log.debug('Motion gate closed, LF_Lanenet reports movement decision only.')
                    publish_scene(
                        'LF_Lanenet',
                        status='would_trigger',
                        data={'lane': {'would_action': intended_action, 'decision_enabled': True, 'motion_enabled': False}},
                    )
                    publish_decision(
                        'lane',
                        'LF_Lanenet',
                        status='would_trigger',
                        action=intended_action,
                        data={'decision_enabled': True, 'motion_enabled': False},
                    )
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue
                if pedestrian_blocked:
                    motion_started = False
                    log.info('Pedestrian stop gate is active, LaneNet pauses motor commands.')
                    publish_scene(
                        'LF_Lanenet',
                        status='paused_for_pedestrian',
                        data={'lane': {'would_action': intended_action, 'decision_enabled': True, 'motion_enabled': True}},
                    )
                    publish_decision(
                        'lane',
                        'LF_Lanenet',
                        status='paused_for_pedestrian',
                        action=intended_action,
                        data={'decision_enabled': True, 'motion_enabled': True, 'pedestrian_blocked': True},
                    )
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue
                sign_action_active = is_sign_action_active()
                if sign_action_active:
                    publish_scene(
                        'LF_Lanenet',
                        status='paused_for_sign_action',
                        data={'lane': {'would_action': intended_action, 'decision_enabled': True, 'motion_enabled': True}},
                    )
                    publish_decision(
                        'lane',
                        'LF_Lanenet',
                        status='paused_for_sign_action',
                        action=intended_action,
                        data={'decision_enabled': True, 'motion_enabled': True},
                    )
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue
                if should_resume_after_pedestrian(pedestrian_blocked, sign_action_active):
                    if consume_pedestrian_resume():
                        log.info('Pedestrian left the stop region; LaneNet resumes control.')
                        self.ctrl.execute(Start())
                        motion_started = True
                        publish_decision(
                            'lane',
                            'LF_Lanenet',
                            status='pedestrian_resume_start',
                            action='resume_start',
                            data={'decision_enabled': True, 'motion_enabled': True, 'pedestrian_blocked': False},
                        )
                        frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                        continue
                if not motion_started:
                    self.ctrl.execute(Start())
                    motion_started = True
                elif consume_motion_resume():
                    log.info('LaneNet resuming after a completed sign action.')
                    self.ctrl.execute(Start())
                    publish_decision(
                        'lane',
                        'LF_Lanenet',
                        status='resume_start',
                        action='resume_start',
                        data={'decision_enabled': True, 'motion_enabled': True},
                    )
                    frame = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=self.broadcaster.buf)
                    continue
                publish_decision(
                    'lane',
                    'LF_Lanenet',
                    status='triggered',
                    action=intended_action,
                    data={'decision_enabled': True, 'motion_enabled': True},
                )
                
                
                if intended_action == 'advance':
                    self.ctrl.execute(Advance(speed=STRAIGHT_SPEED))
                    #self.ctrl.execute(Sleep(0.3))
                elif intended_action == 'turn_right':
                    # self.ctrl.execute(SpinClockwise(speed=30))
                    self.ctrl.execute(TurnRight(speed=TURN_SPEED, degree=turn_strength))
                    #self.ctrl.execute(Sleep(sleep_time))
                else:
                    # self.ctrl.execute(SpinAntiClockwise(speed=30))
                    self.ctrl.execute(TurnLeft(speed=TURN_SPEED, degree=turn_strength))
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
