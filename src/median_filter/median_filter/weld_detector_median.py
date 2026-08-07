import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from scipy.signal import savgol_filter, find_peaks
from scipy.ndimage import median_filter


class WeldDetectorMedian(Node):
    def __init__(self):
        super().__init__('weld_detector_median')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.angle_pub  = self.create_publisher(Float32, '/best_angle', 10)
        self.status_pub = self.create_publisher(String,  '/weld_status', 10)
        self.raw_angle_pub  = self.create_publisher(Float32, '/raw_angle', 10)

        self.roi_start = 313
        self.roi_end   = 421   # 109 steps

        self.sg_order    = 3
        self.sg_framelen = 9    # smooth 4 จุดซ้าย-ขวา
        self.med_window  = 61   # background window

        self.min_prominence       = 0.0015
        self.min_height_threshold = 0.0035
        self.max_width            = 40

        self.last_valid_angle     = float('nan')
        self.last_known_angle     = float('nan')
        self.missed_count         = 0
        self.reset_threshold      = 10
        self.angle_diff_threshold = math.radians(3.0)  # เข้มกว่าเดิมมาก

        self.relock_candidate_angle   = float('nan')
        self.relock_candidate_count   = 0
        self.relock_confirm_threshold = 5
        self.relock_tolerance         = math.radians(3.0)

        # v4 direction-consistency gate (validated via backtest v1-v4, confirm_threshold=6)
        self.streak_sign = 0
        self.streak_len  = 0
        self.consistency_confirm_threshold = 6
        self.sign_eps = math.radians(0.05)

        self.coast_count = 0
        self.coast_max    = 0   # หลุดได้กี่เฟรมก่อนยอมแพ้ (ปรับได้)

        self.angle_history = []
        self.history_size  = 5

        self.center_avg       = 0.093
        self.lateral_scale    = 0.0
        self.lateral_deadband = 0.004
        self.heading_offset   = math.radians(0.0)

        self.last_scan_time    = self.get_clock().now()
        self.scan_timeout_sec  = 3.0
        self.timer             = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered = False

        self.get_logger().info(
            'Weld Detector (Senior + Lateral + Re-lock Gate + Coast) Started.'
        )

    def publish_status(self, status):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_nan(self, reason='', status='NO_WELD'):
        out      = Float32()
        out.data = float('nan')
        self.angle_pub.publish(out)
        self.publish_status(status)
        if reason:
            self.get_logger().warn(reason)

    def reset_relock_gate(self):
        self.relock_candidate_angle = float('nan')
        self.relock_candidate_count = 0

    def check_scan_timeout(self):
        dt = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if dt > self.scan_timeout_sec and not self.timeout_triggered:
            self.last_valid_angle  = float('nan')
            self.last_known_angle  = float('nan')
            self.missed_count      = 0
            self.coast_count       = 0
            self.reset_relock_gate()
            self.timeout_triggered = True
            self.publish_nan(f'No scan {dt:.1f}s', status='TIMEOUT')

    def index_to_angle(self, index, angle_min, angle_increment):
        return angle_min + index * angle_increment

    def is_valid_weld(self, current_angle):
        if math.isnan(self.last_known_angle):
            return True
        return abs(current_angle - self.last_known_angle) < self.angle_diff_threshold

    def scan_callback(self, msg: LaserScan):

        self.last_scan_time    = self.get_clock().now()
        self.timeout_triggered = False

        raw = np.array(
            msg.ranges[self.roi_start:self.roi_end + 1],
            dtype=float
        )
        raw[np.isinf(raw)] = msg.range_max
        raw[np.isnan(raw)] = 0.0

        if len(raw) < 50:
            self.publish_nan('ROI too short', status='ERROR')
            return

        try:
            smooth = savgol_filter(raw, self.sg_framelen, self.sg_order)
            background = median_filter(smooth, size=self.med_window, mode='nearest')
            flattened = background - smooth

            roi_avg       = float(np.mean(raw))
            lateral_error = roi_avg - self.center_avg

            if abs(lateral_error) > self.lateral_deadband:
                lateral_angle = lateral_error * self.lateral_scale
            else:
                lateral_angle = 0.0

            peaks, props = find_peaks(
                flattened,
                prominence=self.min_prominence,
                width=0
            )

            found_weld = False
            best_angle = float('nan')

            was_locked = not math.isnan(self.last_known_angle)

            if len(peaks) > 0:
                prominences = props['prominences']
                widths      = props['widths']

                sorted_idx = np.argsort(prominences)[::-1]
                candidates = []
                for k in range(min(len(sorted_idx), 3)):
                    idx            = sorted_idx[k]
                    local_idx      = int(peaks[idx])
                    current_width  = float(widths[idx])
                    current_height = float(flattened[local_idx])
                    global_idx     = self.roi_start + local_idx
                    current_angle  = self.index_to_angle(
                        global_idx, msg.angle_min, msg.angle_increment
                    )
                    loc_valid    = self.is_valid_weld(current_angle)
                    height_valid = current_height >= self.min_height_threshold
                    if current_width <= self.max_width and loc_valid and height_valid:
                        candidates.append(current_angle)

                if candidates:
                    if math.isnan(self.last_known_angle):
                        raw_best = min(candidates, key=lambda a: abs(a))
                    else:
                        raw_best = min(candidates, key=lambda a: abs(a - self.last_known_angle))

                    # --- v4 direction-consistency gate ---
                    if math.isnan(self.last_known_angle):
                        accept = True
                    else:
                        delta = raw_best - self.last_known_angle
                        sign = 1 if delta > self.sign_eps else (-1 if delta < -self.sign_eps else 0)
                        if sign != 0 and sign == self.streak_sign:
                            self.streak_len += 1
                        elif sign != 0:
                            self.streak_sign, self.streak_len = sign, 1
                        else:
                            self.streak_sign, self.streak_len = 0, 0
                        accept = self.streak_len < self.consistency_confirm_threshold

                    if accept:
                        best_angle = raw_best
                        found_weld            = True
                        self.last_valid_angle = best_angle
                        self.last_known_angle = best_angle
                        self.missed_count     = 0
                        self.coast_count       = 0
                        raw_msg = Float32()
                        raw_msg.data = float(best_angle)
                        self.raw_angle_pub.publish(raw_msg)
                    else:
                        # สงสัยว่า drift สะสม -> freeze ที่ค่าเดิม
                        best_angle = self.last_known_angle
                        found_weld = True

            if found_weld and not was_locked:
                if math.isnan(self.relock_candidate_angle):
                    self.relock_candidate_angle = best_angle
                    self.relock_candidate_count = 1
                elif abs(best_angle - self.relock_candidate_angle) < self.relock_tolerance:
                    self.relock_candidate_count += 1
                else:
                    self.relock_candidate_angle = best_angle
                    self.relock_candidate_count = 1

                if self.relock_candidate_count < self.relock_confirm_threshold:
                    self.last_valid_angle = float('nan')
                    self.last_known_angle = float('nan')
                    found_weld            = False
                    best_angle            = float('nan')
                else:
                    self.reset_relock_gate()

            if found_weld and not math.isnan(best_angle):

                combined = best_angle + lateral_angle

                self.angle_history.append(combined)
                if len(self.angle_history) > self.history_size:
                    self.angle_history.pop(0)
                smoothed = float(np.median(self.angle_history))

                self.get_logger().info(
                    f'Weld Found. '
                    f'heading={math.degrees(best_angle):.2f} | '
                    f'lateral={math.degrees(lateral_angle):.2f} | '
                    f'out={math.degrees(smoothed):.2f} deg'
                )

                corrected = smoothed - self.heading_offset
                self.publish_status('WELD_FOUND')
                out      = Float32()
                out.data = float(corrected)
                self.angle_pub.publish(out)

            else:
                self.angle_history = []
                self.missed_count += 1

                if self.missed_count >= self.reset_threshold:
                    self.last_valid_angle  = float('nan')
                    self.last_known_angle  = float('nan')
                    self.missed_count      = 0
                    self.coast_count       = 0
                    self.streak_sign, self.streak_len = 0, 0
                    self.reset_relock_gate()
                    self.publish_nan('No valid weld', status='NO_WELD')
                elif (
                    self.coast_count < self.coast_max
                    and not math.isnan(self.last_valid_angle)
                ):
                    self.coast_count += 1
                    self.publish_status('WELD_FOUND')
                    out      = Float32()
                    out.data = float(self.last_valid_angle)
                    self.angle_pub.publish(out)
                    self.get_logger().info(
                        f'Coasting on last angle '
                        f'({self.coast_count}/{self.coast_max})'
                    )
                else:
                    self.publish_nan('No valid weld', status='NO_WELD')

        except Exception as e:
            self.last_valid_angle  = float('nan')
            self.last_known_angle  = float('nan')
            self.angle_history     = []
            self.coast_count       = 0
            self.reset_relock_gate()
            self.publish_nan(f'Error: {e}', status='ERROR')


def main(args=None):
    rclpy.init(args=args)
    node = WeldDetectorMedian()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
