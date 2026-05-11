import time
import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32

from gpiozero import PWMOutputDevice, DigitalOutputDevice


class CmdVelToMotorClosedLoop(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor')

        self.wheel_base = 0.30
        self.wheel_radius = 0.0365
        self.max_wheel_speed = 0.20
        self.min_pwm = 0.30
        self.cmd_timeout = 0.5

        self.ticks_per_revolution = 400
        self.encoder_sign = -1

        self.kp_speed = 1.5
        self.speed_filter_alpha = 0.8
        self.min_speed_dt = 0.02
        self.max_valid_speed = 0.20

        self.left_scale = 1.00
        self.right_scale = 1.00

        self.left_pwm_pin = 18
        self.left_dir_pin = 17
        self.right_pwm_pin = 19
        self.right_dir_pin = 26

        self.serial_device = '/dev/ttyUSB0'
        self.serial_baudrate = 115200
        self.serial_timeout = 0.01

        self.serial_port = None
        self.right_ticks = 0

        self.last_cmd_time = time.time()

        self.target_left_speed = 0.0
        self.target_right_speed = 0.0

        self.feedforward_left_pwm = 0.0
        self.feedforward_right_pwm = 0.0

        self.left_dir_cmd = True
        self.right_dir_cmd = True

        self.prev_right_ticks = None
        self.prev_speed_time = None
        self.measured_right_speed = 0.0

        self.left_pwm = PWMOutputDevice(self.left_pwm_pin, frequency=1000, initial_value=0.0)
        self.left_dir = DigitalOutputDevice(self.left_dir_pin, initial_value=False)

        self.right_pwm = PWMOutputDevice(self.right_pwm_pin, frequency=1000, initial_value=0.0)
        self.right_dir = DigitalOutputDevice(self.right_dir_pin, initial_value=False)

        self.open_serial_port()

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.right_tick_pub = self.create_publisher(
            Int32,
            '/right_encoder_ticks',
            10
        )

        self.right_speed_pub = self.create_publisher(
            Float32,
            '/right_wheel_speed',
            10
        )

        self.serial_timer = self.create_timer(0.01, self.read_serial_data)
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('cmd_vel_to_motor closed-loop node started')

    def open_serial_port(self):
        try:
            self.serial_port = serial.Serial(
                self.serial_device,
                self.serial_baudrate,
                timeout=self.serial_timeout
            )
            time.sleep(2.0)
            self.get_logger().info(
                f'Connected to Arduino on {self.serial_device} at {self.serial_baudrate} baud'
            )
        except Exception as e:
            self.serial_port = None
            self.get_logger().warn(
                f'Could not open serial port {self.serial_device}: {e}'
            )

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        v = msg.linear.x
        w = msg.angular.z

        self.target_left_speed = v - (w * self.wheel_base / 2.0)
        self.target_right_speed = v + (w * self.wheel_base / 2.0)

        self.feedforward_left_pwm, self.left_dir_cmd = self.velocity_to_pwm(
            self.target_left_speed
        )
        self.feedforward_right_pwm, self.right_dir_cmd = self.velocity_to_pwm(
            self.target_right_speed
        )

        self.feedforward_left_pwm *= self.left_scale
        self.feedforward_right_pwm *= self.right_scale

        self.feedforward_left_pwm = max(0.0, min(self.feedforward_left_pwm, 1.0))
        self.feedforward_right_pwm = max(0.0, min(self.feedforward_right_pwm, 1.0))

    def velocity_to_pwm(self, velocity):
        direction = velocity >= 0.0
        speed = abs(velocity)

        pwm = speed / self.max_wheel_speed if self.max_wheel_speed > 0.0 else 0.0
        pwm = max(0.0, min(pwm, 1.0))

        if pwm > 0.0:
            pwm = self.min_pwm + (1.0 - self.min_pwm) * pwm
            pwm = min(pwm, 1.0)

        return pwm, direction

    def read_serial_data(self):
        if self.serial_port is None:
            return

        try:
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                if not line:
                    continue

                if line.startswith('T:'):
                    line = line[2:]

                try:
                    self.right_ticks = self.encoder_sign * int(line)
                except ValueError:
                    self.get_logger().warn(f'Invalid serial data: "{line}"')

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def compute_right_speed(self):
        current_time = time.time()

        if self.prev_right_ticks is None or self.prev_speed_time is None:
            self.prev_right_ticks = self.right_ticks
            self.prev_speed_time = current_time
            self.measured_right_speed = 0.0
            return

        dt = current_time - self.prev_speed_time

        if dt < self.min_speed_dt:
            return

        delta_ticks = self.right_ticks - self.prev_right_ticks

        delta_rev = delta_ticks / float(self.ticks_per_revolution)
        delta_distance = delta_rev * (2.0 * math.pi * self.wheel_radius)

        raw_speed = delta_distance / dt

        if abs(raw_speed) <= self.max_valid_speed:
            self.measured_right_speed = (
                self.speed_filter_alpha * self.measured_right_speed +
                (1.0 - self.speed_filter_alpha) * raw_speed
            )

        self.prev_right_ticks = self.right_ticks
        self.prev_speed_time = current_time

    def control_loop(self):
        self.compute_right_speed()

        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_left_speed = 0.0
            self.target_right_speed = 0.0
            self.feedforward_left_pwm = 0.0
            self.feedforward_right_pwm = 0.0

        left_pwm_cmd = self.feedforward_left_pwm
        if abs(self.target_left_speed) < 1e-4:
            left_pwm_cmd = 0.0

        right_error = self.target_right_speed - self.measured_right_speed
        right_pwm_cmd = self.feedforward_right_pwm + (self.kp_speed * right_error)

        if abs(self.target_right_speed) < 1e-4:
            right_pwm_cmd = 0.0
            self.measured_right_speed = 0.0

        left_pwm_cmd = max(0.0, min(left_pwm_cmd, 1.0))
        right_pwm_cmd = max(0.0, min(right_pwm_cmd, 1.0))

        self.left_dir.value = self.left_dir_cmd
        self.right_dir.value = not self.right_dir_cmd

        self.left_pwm.value = left_pwm_cmd
        self.right_pwm.value = right_pwm_cmd

        tick_msg = Int32()
        tick_msg.data = self.right_ticks
        self.right_tick_pub.publish(tick_msg)

        speed_msg = Float32()
        speed_msg.data = float(self.measured_right_speed)
        self.right_speed_pub.publish(speed_msg)

    def destroy_node(self):
        try:
            self.left_pwm.value = 0.0
            self.right_pwm.value = 0.0

            self.left_pwm.close()
            self.right_pwm.close()
            self.left_dir.close()
            self.right_dir.close()

            if self.serial_port is not None and self.serial_port.is_open:
                self.serial_port.close()

        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorClosedLoop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()