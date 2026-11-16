#!/usr/bin/env python3
"""Check the SG90-HV stand-in RW end to end with temporary PWM files, without GPIO.

Starts sg90hv_attitude.launch.py twice like the PC / Pi split (control only, hardware only),
publishes poses and manual commands, and checks the pulse written to the fake PWM channel:
manual speed levels, yaw-hold PID sign, leaving PID on pose loss, the hardware watchdog when
the control side stops, and PWM disable on shutdown. See docs/sg90hv-rw.md.
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from airtable_interfaces.msg import ManualMotionCommand, ReactionWheelSpeedCommand
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy

NEUTRAL_NS = 1_500_000
# config/sg90hv.yaml: 1000 / 1500 / 2000 us, no deadband -> 500 us per unit speed.
NS_PER_UNIT = 500_000


def wait_for(node, predicate, timeout=10.0, what='expected ROS/PWM behavior'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError(f'Timed out waiting for {what}')


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def launch(namespace, log_path, **arguments):
    return subprocess.Popen(
        ['ros2', 'launch', 'airtable_bringup', 'sg90hv_attitude.launch.py',
         f'namespace:={namespace}', 'start_joy:=false',
         *(f'{key}:={value}' for key, value in arguments.items())],
        stdout=log_path.open('w'), stderr=subprocess.STDOUT, start_new_session=True)


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    rclpy.init()
    namespace = f'sg90hv_check_{os.getpid()}'
    node = rclpy.create_node(f'sg90hv_check_{os.getpid()}')
    with tempfile.TemporaryDirectory(prefix='airtable-sg90hv-') as directory:
        root = Path(directory)
        pwm = root / 'pwm2'
        pwm.mkdir()
        for name, value in [('enable', '0'), ('period', '0'), ('duty_cycle', '0'),
                            ('polarity', 'normal')]:
            (pwm / name).write_text(value)

        def duty():
            return int((pwm / 'duty_cycle').read_text() or '0')

        def duty_is(speed):
            return lambda: duty() == NEUTRAL_NS + round(speed * NS_PER_UNIT)

        commands = []
        node.create_subscription(
            ReactionWheelSpeedCommand, f'/{namespace}/actuators/reaction_wheels/speed_command',
            commands.append, 10)
        manual = node.create_publisher(
            ManualMotionCommand, f'/{namespace}/commands/manual/motion', 10)
        pose_publisher = node.create_publisher(
            PoseWithCovarianceStamped, f'/{namespace}/measurements/external_pose', 10)
        pose = {'yaw': 0.0, 'on': False}

        def publish_pose():
            if not pose['on']:
                return
            message = PoseWithCovarianceStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'table_1'
            message.pose.pose.orientation.z = math.sin(pose['yaw'] / 2.0)
            message.pose.pose.orientation.w = math.cos(pose['yaw'] / 2.0)
            pose_publisher.publish(message)

        node.create_timer(1.0 / 30.0, publish_pose)

        def press(command):
            manual.publish(ManualMotionCommand(command=command))

        hardware_log = root / 'hardware.log'
        control_log = root / 'control.log'
        hardware = launch(namespace, hardware_log, start_control='false', pwm_path=str(pwm))
        control = None
        try:
            wait_for(node, lambda: (pwm / 'enable').read_text() == '1', 20, 'PWM enable')
            assert (pwm / 'period').read_text() == '20000000'
            assert duty() == NEUTRAL_NS
            control = launch(namespace, control_log, start_hardware='false')
            wait_for(node, lambda: commands and manual.get_subscription_count() > 0, 20,
                     'control side')

            # Manual levels from rw_attitude_sg90hv.yaml: ... -0.1, 0, 0.1, 0.25 ...
            press(ManualMotionCommand.RW_ROTATE_CCW)
            wait_for(node, duty_is(0.1), what='level 0.1')
            press(ManualMotionCommand.RW_ROTATE_CCW)
            wait_for(node, duty_is(0.25), what='level 0.25')
            for _ in range(3):
                press(ManualMotionCommand.RW_ROTATE_CW)
            wait_for(node, duty_is(-0.1), what='level -0.1')
            print('PASS: manual speed levels reach the PWM through the speed command')

            # PID refuses to start without a pose, then holds the yaw at toggle time.
            press(ManualMotionCommand.RW_ATTITUDE_TOGGLE)
            spin_for(node, 0.5)
            pose['on'] = True
            spin_for(node, 0.5)
            assert duty() == NEUTRAL_NS - round(0.1 * NS_PER_UNIT), 'PID started without pose'
            press(ManualMotionCommand.RW_ATTITUDE_TOGGLE)
            wait_for(node, duty_is(0.0), what='PID holding at zero error')
            # Body 0.2 rad CW of the target: turn it CCW by spinning the wheel CW (kp 0.5).
            pose['yaw'] = -0.2
            wait_for(node, duty_is(-0.1), what='PID output for CW error')
            pose['yaw'] = 0.2
            wait_for(node, duty_is(0.1), what='PID output for CCW error')
            press(ManualMotionCommand.RW_ROTATE_CCW)
            spin_for(node, 0.5)
            assert duty_is(0.1)(), 'manual step changed the output while PID was on'
            print('PASS: PID holds the toggle-time yaw with the wheel opposite to the error')

            pose['on'] = False
            wait_for(node, duty_is(0.0), 3, 'PID leaving on pose loss')
            pose['yaw'] = 0.0
            pose['on'] = True
            press(ManualMotionCommand.RW_ROTATE_CCW)
            wait_for(node, duty_is(0.1), what='manual after pose loss')
            print('PASS: pose loss leaves PID and stops the wheel; manual works again')

            stop(control)
            wait_for(node, duty_is(0.0), 3, 'hardware watchdog')
            assert (pwm / 'enable').read_text() == '1'
            print('PASS: hardware stops the servo when the control side goes away')
        except Exception:
            print(hardware_log.read_text())
            if control_log.exists():
                print(control_log.read_text())
            raise
        finally:
            if control is not None:
                stop(control)
            stop(hardware)
            node.destroy_node()
            rclpy.shutdown()
        assert hardware.returncode == 0, hardware_log.read_text()
        assert 'process has died' not in hardware_log.read_text(), hardware_log.read_text()
        assert (pwm / 'enable').read_text() == '0', 'PWM was not disabled on shutdown'
        print('PASS: hardware shutdown disables PWM')


if __name__ == '__main__':
    main()
