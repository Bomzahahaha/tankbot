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
            LaserScan, '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.angle_pub  = self.create_publisher(Float32, '/best_angle', 10)
        self.status_pub = self.create_publisher(String,  '/weld_status', 10)

        # ── ROI ──────────────────────────────────────────
        self.roi_start = 355
        self.roi_end   = 408   # 91 steps

        # ── Filter ───────────────────────────────────────
        self.sg_order    = 3
        self.sg_framelen = 7
        self.med_window  = 31  # ต้องน้อยกว่า ROI (91)

        # ── Peak ─────────────────────────────────────────
        self.min_prominence = 0.003
        self.max_width      = 20

        # ── T-Junction ───────────────────────────────────
        self.t_junction_ratio             = 0.6
        self.t_junction_count             = 0
        self.t_junction_confirm_threshold = 15
        self.t_junction_min_separation    = 30
        self.t_junction_min_prominence    = 0.005

        # ── Tracking ─────────────────────────────────────
        self.last_valid_angle     = float('nan')
        self.missed_count         = 0
        self.reset_threshold      = 5
        self.angle_diff_threshold = math.radians(8.0)

        # ── Timeout ──────────────────────────────────────
        self.last_scan_time    = self.get_clock().now()
        self.scan_timeout_sec  = 3.0
        self.timer             = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered = False
        self.system_stopped    = False

        self.get_logger().info('Weld Detector (Median - Fixed) Started.')

    # ─────────────────────────────────────────────────────
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_nan(self, reason='', status='NO_WELD'):
        msg_out = Float32()
        msg_out.data = float('nan')
        self.angle_pub.publish(msg_out)
        self.publish_status(status)
        if reason:
            self.get_logger().warn(reason)

    def check_scan_timeout(self):
        if self.system_stopped:
            return
        dt = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if dt > self.scan_timeout_sec and not self.timeout_triggered:
            self.last_valid_angle = float('nan')
            self.missed_count     = 0
            self.t_junction_count = 0
            self.timeout_triggered = True
            self.publish_nan(f'No scan {dt:.1f}s', status='TIMEOUT')

    def index_to_angle(self, index, angle_min, angle_increment):
        return angle_min + index * angle_increment

    # ─────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        if self.system_stopped:
            return

        self.last_scan_time    = self.get_clock().now()
        self.timeout_triggered = False

        raw = np.array(
            msg.ranges[self.roi_start:self.roi_end + 1],
            dtype=float
        )
        raw[np.isinf(raw)] = msg.range_max
        raw[np.isnan(raw)] = 0.0

        if len(raw) < self.sg_framelen + 2:
            self.publish_nan('ROI too short', status='ERROR')
            return

        try:
            # ── Signal Processing ─────────────────────────
            smooth     = savgol_filter(raw, self.sg_framelen, self.sg_order)
            background = median_filter(smooth, size=self.med_window, mode='nearest')
            signal     = background - smooth   # peak = นูนขึ้น

            peaks, props = find_peaks(
                signal,
                prominence=self.min_prominence,
                width=0
            )

            # ── T-Junction Check ──────────────────────────
            if len(peaks) >= 2:
                prom = props['prominences']
                si   = np.argsort(prom)[::-1]

                top1  = float(prom[si[0]])
                top2  = float(prom[si[1]])
                sep   = abs(int(peaks[si[0]]) - int(peaks[si[1]]))
                ratio = top2 / top1 if top1 > 0 else 0.0

                self.get_logger().warn(
                    f'T1={top1:.4f} | T2={top2:.4f} | '
                    f'Ratio={ratio:.2f} | Sep={sep}'
                )

                if (ratio >= self.t_junction_ratio
                        and sep   >= self.t_junction_min_separation
                        and top1  >= self.t_junction_min_prominence
                        and top2  >= self.t_junction_min_prominence):

                    self.t_junction_count += 1
                    self.get_logger().warn(
                        f'Possible T-junction '
                        f'({self.t_junction_count}/'
                        f'{self.t_junction_confirm_threshold})'
                    )
                    if self.t_junction_count >= self.t_junction_confirm_threshold:
                        self.system_stopped = True
                        self.publish_nan('T-junction confirmed → STOP',
                                         status='T_JUNCTION')
                        return
                else:
                    self.t_junction_count = 0
            else:
                self.t_junction_count = 0

            # ── Normal Weld Detection (แก้ใหม่ทั้งหมด) ───
            found_weld = False
            best_angle = float('nan')

            if len(peaks) > 0:
                prom   = props['prominences']
                widths = props['widths']

                # สร้าง candidate list กรอง width ก่อน
                candidates = []
                for i in range(len(peaks)):
                    if float(widths[i]) <= self.max_width:
                        local_idx  = int(peaks[i])
                        global_idx = self.roi_start + local_idx
                        angle      = self.index_to_angle(
                            global_idx,
                            msg.angle_min,
                            msg.angle_increment
                        )
                        candidates.append({
                            'angle': angle,
                            'prom':  float(prom[i]),
                        })

                if len(candidates) > 0:
                    if math.isnan(self.last_valid_angle):
                        # ── ยังไม่มี history → เลือก prominence สูงสุด
                        candidates.sort(key=lambda x: x['prom'], reverse=True)
                        best_angle = candidates[0]['angle']

                    else:
                        # ── มี history → เลือกที่ใกล้ last_valid_angle มากสุด
                        candidates.sort(
                            key=lambda x: abs(x['angle'] - self.last_valid_angle)
                        )
                        nearest       = candidates[0]
                        nearest_diff  = abs(nearest['angle'] - self.last_valid_angle)

                        if nearest_diff < self.angle_diff_threshold:
                            best_angle = nearest['angle']
                        # ถ้าทุก peak ไกลเกิน threshold → best_angle = nan

            # ── ผล ───────────────────────────────────────
            if not math.isnan(best_angle):
                found_weld            = True
                self.last_valid_angle = best_angle
                self.missed_count     = 0
                self.t_junction_count = 0

                self.get_logger().info(
                    f'Weld Found. Angle: {math.degrees(best_angle):.2f} deg'
                )
                self.publish_status('WELD_FOUND')
                out      = Float32()
                out.data = float(best_angle)
                self.angle_pub.publish(out)

            else:
                self.missed_count += 1
                if self.missed_count >= self.reset_threshold:
                    self.last_valid_angle = float('nan')
                    self.missed_count     = 0
                self.publish_nan('No valid weld', status='NO_WELD')

        except Exception as e:
            self.last_valid_angle = float('nan')
            self.t_junction_count = 0
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
