import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class SeamTrackerPID(Node):
    def __init__(self):
        super().__init__('seam_tracker_pid')

        # ── Mode ─────────────────────────────────
        self.tank_mode = False

        # ── PID gains ────────────────────────────
        self.kp = 2.5
        self.ki = 0.0
        self.kd = 0.15

        # ── Motion ───────────────────────────────
        self.max_linear_speed = 0.035
        self.min_linear_speed = 0.005
        self.max_turn_speed   = 0.30
        self.stop_angle_rad   = math.radians(10.0)

        # ── Deadband ─────────────────────────────
        self.deadband_rad = math.radians(0.2)

        # ── Low-pass filter ───────────────────────
        self.filter_alpha   = 0.2
        self.filtered_error = None

        # ── Timeout ──────────────────────────────
        self.angle_timeout = 0.3

        # ── NO_WELD: หยุดถ้าไม่เจอเส้น 3 frames ─
        self.no_weld_count     = 0
        self.no_weld_threshold = 3
        self.no_weld_stopped   = False

        # ── PID state ────────────────────────────
        self.previous_error = 0.0
        self.integral       = 0.0

        self.last_time       = self.get_clock().now()
        self.last_angle_time = self.get_clock().now()

        #Slew-rate limiter
        self.prev_angular_speed = 0.0
        self.max_angular_step = 0.05

        # ── Subscribers ──────────────────────────
        self.angle_subscriber = self.create_subscription(
            Float32,
            '/best_angle',
            self.pid_callback,
            10
        )

        # รับ weld_status เพื่อดู NO_WELD / WELD_FOUND
        self.status_subscriber = self.create_subscription(
            String,
            '/weld_status',
            self.status_callback,
            10
        )

        # ── Publisher ────────────────────────────
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.timer = self.create_timer(0.05, self.check_timeout)

        self.get_logger().info('Seam Tracker PID node started')

    # ── Stop robot ───────────────────────────────
    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x  = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.prev_angular_speed = 0.0

    # ── Timeout check ────────────────────────────
    def check_timeout(self):
        if self.no_weld_stopped:
            self.stop_robot()
            return
        now = self.get_clock().now()
        dt  = (now - self.last_angle_time).nanoseconds / 1e9
        if dt > self.angle_timeout:
            self.stop_robot()

    # ── Status callback ───────────────────────────
    def status_callback(self, msg: String):
        status = msg.data

        if status == 'WELD_FOUND':
            self.no_weld_count   = 0
            self.no_weld_stopped = False  # ← reset ได้เมื่อเจอเส้น

    # ── Linear speed ─────────────────────────────
    def calculate_linear_speed(self, error):
        error_abs = abs(error)
        if error_abs >= self.stop_angle_rad:
            return 0.0
        ratio = error_abs / self.stop_angle_rad
        speed = self.max_linear_speed * (1.0 - ratio) ** 2
        return max(speed, self.min_linear_speed)

    def calculate_linear_speed_tank(self, error):
        if abs(error) >= self.stop_angle_rad:
            return 0.0
        return self.min_linear_speed

    def reset_filter(self):
        self.filtered_error = None

    # ── PID callback ─────────────────────────────
    def pid_callback(self, angle_msg: Float32):

        if self.no_weld_stopped:
            self.stop_robot()
            return

        raw_error = angle_msg.data
        self.last_angle_time = self.get_clock().now()

        # ── NO_WELD: NaN → หยุด 3 frames ─────────
        if math.isnan(raw_error) or math.isinf(raw_error):
            self.no_weld_count += 1
            self.get_logger().warn(
                f'No seam detected '
                f'({self.no_weld_count}/{self.no_weld_threshold})'
            )
            self.stop_robot()

            if self.no_weld_count >= self.no_weld_threshold:
                self.no_weld_stopped = True  # ← เพิ่ม flag
                self.get_logger().warn('Confirmed NO_WELD → STOP')
                self.integral       = 0.0
                self.previous_error = 0.0
                self.reset_filter()
            return

        # ── Low-pass filter ───────────────────────
        if self.filtered_error is None:
            self.filtered_error = raw_error
        else:
            self.filtered_error = (
                self.filter_alpha * self.filtered_error +
                (1.0 - self.filter_alpha) * raw_error
            )

        error = self.filtered_error

        # ── Time ──────────────────────────────────
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.0:
            dt = 1e-3

        # ── PID ───────────────────────────────────
        p = self.kp * error

        self.integral += error * dt
        self.integral  = max(min(self.integral, 1.0), -1.0)
        i = self.ki * self.integral

        derivative = (error - self.previous_error) / dt
        d = self.kd * derivative

        angular_speed = -(p + i + d)
        angular_speed = max(
            min(angular_speed, self.max_turn_speed),
            -self.max_turn_speed
        )

        angular_speed = max(
            min(angular_speed, self.max_turn_speed),
            -self.max_turn_speed
        )

        angular_speed = max(
            min(angular_speed, self.prev_angular_speed + self.max_angular_step),
            self.prev_angular_speed - self.max_angular_step
        )

        # ── Deadband ──────────────────────────────
        raw_in_db      = abs(raw_error) < self.deadband_rad
        filtered_in_db = abs(error)     < self.deadband_rad

        if raw_in_db or filtered_in_db:
            angular_speed       = 0.0
            self.integral       = 0.0
            self.previous_error = 0.0
        else:
            self.previous_error = error

        self.prev_angular_speed = angular_speed

        # ── Linear speed ──────────────────────────
        if self.tank_mode:
            linear_speed = self.calculate_linear_speed_tank(error)
        else:
            linear_speed = self.calculate_linear_speed(error)

        if raw_in_db or filtered_in_db:
            linear_speed = (
                self.min_linear_speed if self.tank_mode
                else self.max_linear_speed
            )

        linear_speed = max(0.0, linear_speed)

        # ── Publish ───────────────────────────────
        twist_msg = Twist()
        twist_msg.linear.x  = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)

        mode_name = 'TANK' if self.tank_mode else 'PAPER'
        self.get_logger().info(
            f'mode={mode_name} | '
            f'raw={math.degrees(raw_error):.2f} deg | '
            f'filtered={math.degrees(error):.2f} deg | '
            f'linear={linear_speed:.3f} | '
            f'angular={angular_speed:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SeamTrackerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
