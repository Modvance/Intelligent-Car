import copy
import math
import threading
import time


def _default_state(mode):
    return {
        "updated_at": time.time(),
        "system": {
            "running": True,
            "mode": mode,
            "motion_enabled": False,
            "decision_enabled": False,
        },
        "camera": {"status": "running", "width": 0, "height": 0, "fps": 0},
        "controller": {
            "state": [0, 0, 0, 0, 93, 162, -12345],
            "speed": 0,
            "servo_angle": [93, 162],
            "last_action": "",
            "last_result": "",
            "serial_port": "",
            "latency_ms": 0.0,
        },
        "motor_control": {
            "mode": "encoder_pid", "drive_mode": "closed_loop_encoder_pid",
            "enabled": False, "fresh": False, "age_ms": None,
            "data_source": "esp32_encoder", "wheels": [], "history": [],
        },
        "scene": {"name": "", "status": "idle", "lane": {}, "detections": []},
        "scenes": {
            "LF_Lanenet": {"name": "LF_Lanenet", "status": "idle", "lane": {}},
            "Helper": {"name": "Helper", "status": "idle", "detections": [], "helper": {}},
        },
        "decision": {
            "lane": {"source": "DecisionNode", "status": "idle", "action": "", "updated_at": 0},
            "sign": {"source": "DecisionNode", "status": "idle", "action": "", "updated_at": 0},
        },
        "events": [],
    }


def _finite_or_none(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class LegacyMonitorState:
    def __init__(self, mode="ros2", event_limit=80):
        self._state = _default_state(mode)
        self._event_limit = int(event_limit)
        self._lock = threading.RLock()

    def _touch(self):
        self._state["updated_at"] = time.time()

    def _event(self, event_type, message):
        self._state["events"].append({"time": time.time(), "type": event_type, "message": message})
        self._state["events"] = self._state["events"][-self._event_limit :]

    def update_car(self, motor, servo, result, action, serial_port, motion_enabled, latency_ms):
        with self._lock:
            motor_values = [int(value) for value in motor]
            servo_values = [int(value) for value in servo]
            values = motor_values + servo_values + [-12345]
            controller = self._state["controller"]
            controller.update(
                {
                    "state": values,
                    "speed": max((abs(value) for value in motor_values), default=0),
                    "servo_angle": servo_values,
                    "last_action": action,
                    "last_result": result,
                    "serial_port": serial_port,
                    "latency_ms": float(latency_ms),
                }
            )
            self._state["system"]["motion_enabled"] = bool(motion_enabled)
            self._touch()

    def update_lane(
        self,
        steering,
        filtered,
        inference_ms,
        lanes,
        heading,
        lateral,
        lateral_error,
        lane_center,
        lane_target,
        turn_strength,
        intended_action,
    ):
        with self._lock:
            lane = {
                "raw_count": len(lanes),
                "kept_count": len(lanes),
                "lanes": [
                    {"k": float(k), "b": float(b), "confidence": float(confidence)}
                    for k, b, confidence in lanes
                ],
                "steering_command": _finite_or_none(steering),
                "heading_steering": _finite_or_none(heading),
                "lateral_steering": _finite_or_none(lateral),
                "lateral_error_px": _finite_or_none(lateral_error),
                "lane_center_x": _finite_or_none(lane_center),
                "lane_target_x": _finite_or_none(lane_target),
                "filtered_steering": _finite_or_none(filtered),
                "turn_strength": _finite_or_none(turn_strength),
                "intended_action": str(intended_action),
                "inference_time": _finite_or_none(inference_ms),
            }
            scene = self._state["scenes"]["LF_Lanenet"]
            scene.update({"status": "running", "lane": lane})
            self._state["scene"] = copy.deepcopy(scene)
            self._touch()

    def update_signs(self, detections):
        with self._lock:
            mapped = [
                {"cate": str(label), "score": float(score), "box": [int(value) for value in box]}
                for label, score, box in detections
            ]
            scene = self._state["scenes"]["Helper"]
            scene.update({"status": "running", "detections": mapped})
            self._state["scene"] = copy.deepcopy(scene)
            self._touch()

    def update_decision(
        self,
        enabled,
        pedestrian_blocked,
        action_active,
        status,
        action,
        turn_count,
        right_turn_count,
        turnaround_count,
    ):
        with self._lock:
            now = time.time()
            self._state["system"]["decision_enabled"] = bool(enabled)
            lane_status = "paused_for_pedestrian" if pedestrian_blocked else status
            self._state["decision"]["lane"].update(
                {"status": lane_status, "action": action, "updated_at": now}
            )
            self._state["decision"]["sign"].update(
                {
                    "status": "running" if enabled else "decision_locked",
                    "action": action if action_active else "",
                    "updated_at": now,
                    "turn_count": int(turn_count),
                    "right_turn_count": int(right_turn_count),
                    "turnaround_count": int(turnaround_count),
                    "pedestrian_blocked": bool(pedestrian_blocked),
                }
            )
            self._touch()

    def update_motor_control(self, snapshot):
        with self._lock:
            self._state["motor_control"] = copy.deepcopy(snapshot)
            self._touch()

    def update_motor_telemetry(self, msg):
        """Translate the real ESP32 feedback topic into monitor JSON state."""
        now = time.time()
        wheels = []
        for index in range(4):
            flags = int(msg.flags[index])
            wheels.append({
                "name": f"M{index + 1}",
                "target_rpm": float(msg.target_rpm[index]),
                "measured_rpm": float(msg.measured_rpm[index]),
                "tick_delta": int(msg.tick_delta[index]),
                "pwm": int(msg.pwm[index]),
                "error": float(msg.error[index]),
                "p_term": float(msg.p_term[index]),
                "i_term": float(msg.i_term[index]),
                "d_term": float(msg.d_term[index]),
                "saturated": bool(flags & 0x01),
                "start_pulse": bool(flags & 0x02),
                "flags": flags,
            })
        snapshot = {
            "mode": "encoder_pid",
            "drive_mode": "closed_loop_encoder_pid",
            "enabled": True,
            "fresh": bool(msg.fresh),
            "age_ms": 0,
            "data_source": "esp32_encoder",
            "sequence": int(msg.sequence),
            "period_ms": int(msg.period_ms),
            "wheels": wheels,
            "history": [{"time": now, "wheels": [
                {"target_rpm": wheel["target_rpm"], "measured_rpm": wheel["measured_rpm"], "pwm": wheel["pwm"]}
                for wheel in wheels
            ]}],
        }
        self.update_motor_control(snapshot)

    def update_camera(self, width, height, fps):
        with self._lock:
            self._state["camera"].update(
                {"status": "running", "width": int(width), "height": int(height), "fps": int(fps)}
            )
            self._touch()

    def add_event(self, event_type, message):
        with self._lock:
            self._event(event_type, message)
            self._touch()

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)
