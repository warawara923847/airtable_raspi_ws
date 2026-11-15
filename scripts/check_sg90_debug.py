#!/usr/bin/env python3
"""Check ROS RW control and PWM fault handling using temporary files, without GPIO."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from airtable_interfaces.msg import ManualMotionCommand, ReactionWheelCommand, ReactionWheelState
from ament_index_python.packages import get_package_prefix
import rclpy


def wait_for(node, predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError('Timed out waiting for expected ROS/PWM behavior')


def check_phase(node, root, control):
    phase = 'control' if control else 'direct'
    namespace = f'sg90_check_{os.getpid()}_{phase}'
    pwm = root / phase
    pwm.mkdir()
    for name, value in [('enable', '0'), ('period', '0'), ('duty_cycle', '0'),
                        ('polarity', 'normal')]:
        (pwm / name).write_text(value)
    states = []
    subscription = node.create_subscription(
        ReactionWheelState, f'/{namespace}/actuators/reaction_wheels/state', states.append, 10,
    )
    topic = (f'/{namespace}/commands/manual/motion' if control
             else f'/{namespace}/actuators/reaction_wheels/command')
    publisher = node.create_publisher(
        ManualMotionCommand if control else ReactionWheelCommand, topic, 10,
    )
    log_path = root / f'{phase}.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'airtable_bringup', 'sg90_debug.launch.py',
            f'namespace:={namespace}', f'start_control:={str(control).lower()}',
            'start_joy:=false', 'backend:=sysfs', f'pwm_path:={pwm}',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            wait_for(node, lambda: bool(states) and publisher.get_subscription_count() > 0, 20)
            assert states[-1].names == ['rw_1']
            assert states[-1].stamp.sec > 0
            assert (pwm / 'period').read_text() == '20000000'
            assert (pwm / 'enable').read_text() == '1'

            def pulse():
                return int((pwm / 'duty_cycle').read_text() or '0')

            if control:
                publisher.publish(ManualMotionCommand(command=ManualMotionCommand.RW_ROTATE_CCW))
                wait_for(node, lambda: pulse() > 1500000 and states[-1].commanded_torques[0] > 0)
                wait_for(node, lambda: abs(states[-1].angular_velocities[0] - 5.0) < 0.05)
                publisher.publish(ManualMotionCommand(command=ManualMotionCommand.RW_ROTATE_CW))
                wait_for(node, lambda: pulse() < 1500000)
                wait_for(node, lambda: abs(states[-1].angular_velocities[0]) < 0.05)
                publisher.publish(ManualMotionCommand(command=ManualMotionCommand.RW_ROTATE_CW))
                wait_for(node, lambda: abs(states[-1].angular_velocities[0] + 5.0) < 0.05)
                print('PASS: controller state bootstrap, CCW/CW, virtual speed convergence')
            else:
                for torque, expected in [(0.005, 1600000), (-0.005, 1400000),
                                         (1.0, 1700000), (-1.0, 1300000)]:
                    publisher.publish(ReactionWheelCommand(names=['rw_1'], torques=[torque]))
                    wait_for(node, lambda: pulse() == expected)
                # Invalid messages must not apply an output or refresh the watchdog.
                for message in [
                    ReactionWheelCommand(names=['rw_1'], torques=[]),
                    ReactionWheelCommand(names=['rw_1'], torques=[float('nan')]),
                    ReactionWheelCommand(names=['rw_1', 'rw_1'], torques=[0.01, 0.01]),
                    ReactionWheelCommand(names=['unknown'], torques=[0.01]),
                ]:
                    publisher.publish(message)
                wait_for(node, lambda: pulse() == 1500000, 2)
                wait_for(node, lambda: list(states[-1].commanded_torques) == [0.0])
                assert list(states[-1].applied_torques) == [0.0]
                print('PASS: torque mapping, clamping, invalid messages, 500 ms watchdog')
                # A missing attribute mimics a runtime PWM I/O failure.
                (pwm / 'duty_cycle').unlink()
                wait_for(node, lambda: (pwm / 'enable').read_text() == '0')
                publisher.publish(ReactionWheelCommand(names=['rw_1'], torques=[0.01]))
                received = len(states)
                wait_for(node, lambda: len(states) > received
                         and list(states[-1].commanded_torques) == [0.0])
                assert (pwm / 'enable').read_text() == '0'
                print('PASS: PWM I/O fault disables output and latches until restart')
        except Exception:
            print(log_path.read_text())
            raise
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_publisher(publisher)
            node.destroy_subscription(subscription)
    assert process.returncode == 0, log_path.read_text()
    assert 'process has died' not in log_path.read_text(), log_path.read_text()
    assert (pwm / 'enable').read_text() == '0', 'PWM was not disabled on shutdown'
    print(f'PASS: {phase} shutdown disables PWM')


def check_sigterm(node, root):
    pwm = root / 'control'
    executable = (Path(get_package_prefix('airtable_hardware'))
                  / 'lib' / 'airtable_hardware' / 'airtable_hardware')
    with (root / 'sigterm.log').open('w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-r', f'__ns:=/sg90_sigterm_{os.getpid()}',
            '-p', 'rw.enabled:=true', '-p', 'rw.backend:=sysfs', '-p', f'rw.pwm_path:={pwm}',
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(node, lambda: (pwm / 'enable').read_text() == '1')
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=5) == 0
            assert (pwm / 'enable').read_text() == '0'
            print('PASS: SIGTERM exits normally and disables PWM')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()


def main():
    rclpy.init()
    node = rclpy.create_node(f'sg90_check_{os.getpid()}')
    try:
        with tempfile.TemporaryDirectory(prefix='airtable-sg90-') as directory:
            root = Path(directory)
            check_phase(node, root, control=False)
            check_phase(node, root, control=True)
            check_sigterm(node, root)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
