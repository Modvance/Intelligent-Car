import unittest
from pathlib import Path


SKETCH = Path(__file__).resolve().parents[1] / "car_ctrl_esp32_pid.ino"


class PidFirmwareContractTest(unittest.TestCase):
    def source(self):
        return SKETCH.read_text(encoding="utf-8")

    def test_pid_sketch_keeps_legacy_command_packet_and_declares_telemetry(self):
        source = self.source()

        self.assertIn("PACKET_SHORTS = 7", source)
        self.assertIn("CHECK_VAL = -12345", source)
        self.assertIn("TELEMETRY_MAGIC_0 = 0xA5", source)
        self.assertIn("TELEMETRY_MAGIC_1 = 0x5A", source)
        self.assertIn("sendTelemetry", source)

    def test_pid_sketch_has_anti_windup_and_transition_reset(self):
        source = self.source()

        self.assertIn("conditionalIntegrate", source)
        self.assertIn("resetWheelController", source)
        self.assertIn("START_PULSE_MS", source)
        self.assertIn("PID_KD", source)

    def test_pid_sketch_uses_conservative_generic_encoder_defaults(self):
        source = self.source()

        self.assertIn("ENCODER_PULSES_PER_WHEEL_REV = 24000.0f", source)
        self.assertIn("MAX_WHEEL_RPM = 70.0f", source)
        self.assertIn("CONTROL_PERIOD_MS = 50", source)
        self.assertIn("TELEMETRY_PERIOD_MS = 100", source)


if __name__ == "__main__":
    unittest.main()
