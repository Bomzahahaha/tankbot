import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String


class ControlLogger(Node):
    def __init__(self):
        super().__init__('control_logger')

        self.wheel_base = 0.30
        self.start_time = time.time()

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.latest_target_right_speed = 0.0
        self.latest_actual_right_speed = 0.0
        self.latest_ticks = 0

        self.latest_best_angle = float('nan')
        self.latest_best_angle_deg = float('nan')
        self.latest_detected = 0
        self.latest_weld_status = 'UNKNOWN'

        self.errors = []
        self.abs_errors = []
        self.sq_errors = []

        self.total_angle_count = 0
        self.detected_angle_count = 0
        self.angles = []
        self.delta_angles = []
        self.prev_angle = None

        self.weld_found_count = 0
        self.no_weld_count = 0
        self.t_junction_count = 0
        self.unknown_status_count = 0

        os.makedirs('logs', exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = f'logs/control_log_{timestamp}.csv'

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time_sec',
            'cmd_linear_x',
            'cmd_angular_z',
            'target_right_speed_mps',
            'actual_right_speed_mps',
            'error_mps',
            'abs_error_mps',
            'squared_error',
            'right_encoder_ticks',
            'best_angle_rad',
            'best_angle_deg',
            'detected',
            'weld_status',
            'weld_found_count',
            'no_weld_count',
            't_junction_count',
            'unknown_status_count'
        ])

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Float32, '/right_wheel_speed', self.speed_callback, 10)
        self.create_subscription(Int32, '/right_encoder_ticks', self.tick_callback, 10)
        self.create_subscription(Float32, '/best_angle', self.best_angle_callback, 10)
        self.create_subscription(String, '/weld_status', self.weld_status_callback, 10)

        self.log_timer = self.create_timer(0.02, self.log_data)
        self.summary_timer = self.create_timer(1.0, self.print_summary)

        self.get_logger().info(f'Control logger started. Logging to: {self.filename}')

    def cmd_callback(self, msg: Twist):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z
        self.latest_target_right_speed = (
            msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
        )

    def speed_callback(self, msg: Float32):
        self.latest_actual_right_speed = msg.data

    def tick_callback(self, msg: Int32):
        self.latest_ticks = msg.data

    def best_angle_callback(self, msg: Float32):
        angle = msg.data
        self.latest_best_angle = angle
        self.total_angle_count += 1

        detected = not (math.isnan(angle) or math.isinf(angle))
        self.latest_detected = int(detected)

        if detected:
            self.detected_angle_count += 1
            self.latest_best_angle_deg = math.degrees(angle)
            self.angles.append(angle)

            if self.prev_angle is not None:
                self.delta_angles.append(angle - self.prev_angle)

            self.prev_angle = angle
        else:
            self.latest_best_angle_deg = float('nan')
            self.prev_angle = None

    def weld_status_callback(self, msg: String):
        self.latest_weld_status = msg.data

        if msg.data == 'WELD_FOUND':
            self.weld_found_count += 1
        elif msg.data == 'NO_WELD':
            self.no_weld_count += 1
        elif msg.data == 'T_JUNCTION':
            self.t_junction_count += 1
        else:
            self.unknown_status_count += 1

    def log_data(self):
        t = time.time() - self.start_time

        error = self.latest_target_right_speed - self.latest_actual_right_speed
        abs_error = abs(error)
        sq_error = error ** 2

        self.errors.append(error)
        self.abs_errors.append(abs_error)
        self.sq_errors.append(sq_error)

        self.writer.writerow([
            t,
            self.cmd_linear_x,
            self.cmd_angular_z,
            self.latest_target_right_speed,
            self.latest_actual_right_speed,
            error,
            abs_error,
            sq_error,
            self.latest_ticks,
            self.latest_best_angle,
            self.latest_best_angle_deg,
            self.latest_detected,
            self.latest_weld_status,
            self.weld_found_count,
            self.no_weld_count,
            self.t_junction_count,
            self.unknown_status_count
        ])

        self.file.flush()

    def mean(self, data):
        if len(data) == 0:
            return 0.0
        return sum(data) / len(data)

    def std(self, data):
        if len(data) < 2:
            return 0.0

        avg = self.mean(data)
        variance = sum((x - avg) ** 2 for x in data) / (len(data) - 1)
        return math.sqrt(variance)

    def print_summary(self):
        if len(self.errors) == 0:
            return

        mae = self.mean(self.abs_errors)
        rmse = math.sqrt(self.mean(self.sq_errors))
        max_error = max(self.abs_errors)

        detection_rate = 0.0
        if self.total_angle_count > 0:
            detection_rate = (
                self.detected_angle_count / self.total_angle_count
            ) * 100.0

        angle_jitter_deg = math.degrees(self.std(self.angles))
        delta_angle_jitter_deg = math.degrees(self.std(self.delta_angles))

        self.get_logger().info(
            f'File: {self.filename} | '
            f'MAE: {mae:.4f} m/s | '
            f'RMSE: {rmse:.4f} m/s | '
            f'Max Error: {max_error:.4f} m/s | '
            f'Detection Rate: {detection_rate:.2f}% | '
            f'Angle Jitter: {angle_jitter_deg:.3f} deg | '
            f'Delta Jitter: {delta_angle_jitter_deg:.3f} deg | '
            f'WELD: {self.weld_found_count} | '
            f'NO_WELD: {self.no_weld_count} | '
            f'T_JUNCTION: {self.t_junction_count}'
        )

    def destroy_node(self):
        self.print_summary()
        if not self.file.closed:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()