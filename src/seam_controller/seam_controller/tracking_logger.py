import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist


class TrackingLogger(Node):
    def __init__(self):
        super().__init__('tracking_logger')

        self.start_time = time.time()

        self.best_angle = float('nan')
        self.best_angle_deg = float('nan')
        self.detected = 0
        self.weld_status = 'UNKNOWN'

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        self.total_angle_count = 0
        self.detected_angle_count = 0

        self.angles = []
        self.delta_angles = []
        self.prev_angle = None

        self.angular_values = []
        self.linear_values = []

        self.weld_found_count = 0
        self.no_weld_count = 0
        self.t_junction_count = 0
        self.timeout_count = 0
        self.error_count = 0
        self.unknown_status_count = 0

        # Encoder distance tracking
        self.wheel_radius = 0.0365
        self.ticks_per_revolution = 400

        self.right_encoder_ticks = 0
        self.prev_encoder_ticks = None
        self.delta_encoder_ticks = 0
        self.delta_distance_m = 0.0
        self.total_distance_m = 0.0

        os.makedirs('logs', exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = f'logs/tracking_log_{timestamp}.csv'

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time_sec',
            'best_angle_rad',
            'best_angle_deg',
            'detected',
            'weld_status',
            'cmd_linear_x',
            'cmd_angular_z',

            'right_encoder_ticks',
            'delta_encoder_ticks',
            'delta_distance_m',
            'total_distance_m',

            'weld_found_count',
            'no_weld_count',
            't_junction_count',
            'timeout_count',
            'error_count',
            'unknown_status_count'
        ])

        self.create_subscription(Float32, '/best_angle', self.best_angle_callback, 10)
        self.create_subscription(String, '/weld_status', self.weld_status_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Int32, '/right_encoder_ticks', self.encoder_callback, 10)

        self.log_timer = self.create_timer(0.02, self.log_data)
        self.summary_timer = self.create_timer(1.0, self.print_summary)

        self.get_logger().info(f'Tracking logger started. Logging to: {self.filename}')

    def best_angle_callback(self, msg: Float32):
        angle = msg.data
        self.best_angle = angle
        self.total_angle_count += 1

        detected = not (math.isnan(angle) or math.isinf(angle))
        self.detected = int(detected)

        if detected:
            self.detected_angle_count += 1
            self.best_angle_deg = math.degrees(angle)
            self.angles.append(angle)

            if self.prev_angle is not None:
                self.delta_angles.append(angle - self.prev_angle)

            self.prev_angle = angle
        else:
            self.best_angle_deg = float('nan')
            self.prev_angle = None

    def weld_status_callback(self, msg: String):
        self.weld_status = msg.data

        if msg.data == 'WELD_FOUND':
            self.weld_found_count += 1
        elif msg.data == 'NO_WELD':
            self.no_weld_count += 1
        elif msg.data == 'T_JUNCTION':
            self.t_junction_count += 1
        elif msg.data == 'TIMEOUT':
            self.timeout_count += 1
        elif msg.data == 'ERROR':
            self.error_count += 1
        else:
            self.unknown_status_count += 1

    def cmd_callback(self, msg: Twist):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

        self.linear_values.append(msg.linear.x)
        self.angular_values.append(msg.angular.z)

    def encoder_callback(self, msg: Int32):
        current_ticks = msg.data
        self.right_encoder_ticks = current_ticks

        if self.prev_encoder_ticks is None:
            self.prev_encoder_ticks = current_ticks
            self.delta_encoder_ticks = 0
            self.delta_distance_m = 0.0
            return

        self.delta_encoder_ticks = current_ticks - self.prev_encoder_ticks

        distance_per_tick = (
            2.0 * math.pi * self.wheel_radius
        ) / float(self.ticks_per_revolution)

        self.delta_distance_m = self.delta_encoder_ticks * distance_per_tick
        self.total_distance_m += self.delta_distance_m

        self.prev_encoder_ticks = current_ticks

    def log_data(self):
        t = time.time() - self.start_time

        self.writer.writerow([
            t,
            self.best_angle,
            self.best_angle_deg,
            self.detected,
            self.weld_status,
            self.cmd_linear_x,
            self.cmd_angular_z,

            self.right_encoder_ticks,
            self.delta_encoder_ticks,
            self.delta_distance_m,
            self.total_distance_m,

            self.weld_found_count,
            self.no_weld_count,
            self.t_junction_count,
            self.timeout_count,
            self.error_count,
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
        if self.total_angle_count > 0:
            detection_rate = (self.detected_angle_count / self.total_angle_count) * 100.0
        else:
            detection_rate = 0.0

        angle_jitter_deg = math.degrees(self.std(self.angles))
        delta_jitter_deg = math.degrees(self.std(self.delta_angles))

        avg_angular = self.mean([abs(x) for x in self.angular_values])
        max_angular = max([abs(x) for x in self.angular_values], default=0.0)

        avg_linear = self.mean(self.linear_values)
        max_linear = max(self.linear_values, default=0.0)

        self.get_logger().info(
            f'File: {self.filename} | '
            f'Detection Rate: {detection_rate:.2f}% | '
            f'Angle Jitter: {angle_jitter_deg:.3f} deg | '
            f'Delta Jitter: {delta_jitter_deg:.3f} deg/frame | '
            f'Avg |angular.z|: {avg_angular:.3f} rad/s | '
            f'Max |angular.z|: {max_angular:.3f} rad/s | '
            f'Avg linear.x: {avg_linear:.3f} m/s | '
            f'Max linear.x: {max_linear:.3f} m/s | '
            f'Total Distance: {self.total_distance_m:.3f} m | '
            f'WELD: {self.weld_found_count} | '
            f'NO_WELD: {self.no_weld_count} | '
            f'T_JUNCTION: {self.t_junction_count} | '
            f'TIMEOUT: {self.timeout_count} | '
            f'ERROR: {self.error_count}'
        )

    def destroy_node(self):
        self.print_summary()
        if not self.file.closed:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()