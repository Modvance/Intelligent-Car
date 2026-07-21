from smart_car_nodes.protocol import CHECK_VALUE, CommandFrame


def command_to_msg(command: CommandFrame):
    from smart_car_msgs.msg import CarCommand

    msg = CarCommand()
    msg.motor = list(command.motor)
    msg.servo = list(command.servo)
    msg.check_value = int(command.check_value)
    msg.source = command.source
    return msg


def msg_to_command(msg) -> CommandFrame:
    return CommandFrame(
        motor=list(msg.motor),
        servo=list(msg.servo),
        check_value=int(getattr(msg, "check_value", CHECK_VALUE)),
        source=getattr(msg, "source", ""),
    )


def fill_car_state_msg(msg, node, command, result, serial_port, motion_enabled, latency_ms):
    msg.stamp = node.get_clock().now().to_msg()
    msg.motor = list(command.motor)
    msg.servo = list(command.servo)
    msg.check_value = int(command.check_value)
    msg.speed = int(max(abs(value) for value in command.motor))
    msg.action = command.source
    msg.result = result
    msg.serial_port = serial_port
    msg.motion_enabled = bool(motion_enabled)
    msg.latency_ms = float(latency_ms)
    return msg
