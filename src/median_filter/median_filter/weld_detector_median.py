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

        # ══════════════════════════════════════════
        # ROI — ใช้ของรุ่นพี่ (กว้างพอสำหรับ scan)
        # ปรับ roi_start/roi_end ให้ตรงกับ
        # ตำแหน่งเส้นจริงบนถัง
        # ══════════════════════════════════════════
        self.roi_start = 330
        self.roi_end   = 438   # 109 steps

        # ══════════════════════════════════════════
        # FILTER
        # Savitzky-Golay: smooth noise รักษา peak
        # sg_framelen ต้องคี่ และ < roi size
        # Median Filter: หา background ของพื้น
        # med_window ต้องน้อยกว่า roi size
        # ══════════════════════════════════════════
        self.sg_order    = 3
        self.sg_framelen = 15   # smooth 7 จุดซ้าย-ขวา
        self.med_window  = 51   # background window

        # ══════════════════════════════════════════
        # PEAK PARAMETERS
        # min_prominence: peak ต้องโดดเด่นเท่าไหร่
        # min_height:     peak ต้องสูงเท่าไหร่
        # max_width:      peak กว้างได้ไม่เกินนี้
        # ══════════════════════════════════════════
        self.min_prominence       = 0.002
        self.min_height_threshold = 0.004
        self.max_width            = 60

        # ══════════════════════════════════════════
        # T-JUNCTION
        # ratio: peak2/peak1 ต้องสูงกว่านี้
        # separation: สองpeak ต้องห่างกันเท่าไหร่
        # confirm_threshold: ยืนยันกี่ frame
        # ══════════════════════════════════════════
        self.t_junction_ratio             = 0.85
        self.t_junction_count             = 0
        self.t_junction_confirm_threshold = 10
        self.t_junction_min_separation    = 25
        self.t_junction_min_prominence    = 0.008

        # ══════════════════════════════════════════
        # TRACKING
        # angle_diff_threshold: กัน angle กระโดด
        # reset_threshold: miss กี่ frame ค่อย reset
        # ══════════════════════════════════════════
        self.last_valid_angle     = float('nan')
        self.missed_count         = 0
        self.reset_threshold      = 5
        self.angle_diff_threshold = math.radians(3.0)  # เข้มกว่าเดิมมาก

        # ══════════════════════════════════════════
        # ANGLE HISTORY — median smooth 5 frames
        # ลด noise ของ angle output
        # ══════════════════════════════════════════
        self.angle_history = []
        self.history_size  = 5

        # ══════════════════════════════════════════
        # LATERAL CORRECTION (ของคุณ)
        # center_avg: avg range ตอนอยู่กลางเส้น
        # lateral_scale: 1m error → กี่ radian
        # lateral_deadband: ±เท่านี้ไม่ต้องแก้
        # ══════════════════════════════════════════
        self.center_avg       = 0.093
        self.lateral_scale    = 3.5
        self.lateral_deadband = 0.004

        # ══════════════════════════════════════════
        # TIMEOUT
        # ══════════════════════════════════════════
        self.last_scan_time    = self.get_clock().now()
        self.scan_timeout_sec  = 3.0
        self.timer             = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered = False
        self.system_stopped    = False

        self.get_logger().info('Weld Detector (Senior + Lateral) Started.')

    # ════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════
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

    def is_valid_weld(self, current_angle, past_angle):
        if math.isnan(past_angle):
            return True
        return abs(current_angle - past_angle) < self.angle_diff_threshold

    # ════════════════════════════════════════════════
    # MAIN CALLBACK
    # ════════════════════════════════════════════════
    def scan_callback(self, msg: LaserScan):

        if self.system_stopped:
            return

        self.last_scan_time    = self.get_clock().now()
        self.timeout_triggered = False

        # ── Step 1: ตัด ROI ──────────────────────────
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
            # ── Step 2: Savitzky-Golay Filter ────────
            # smooth noise ออก รักษา shape ของ peak
            smooth = savgol_filter(raw, self.sg_framelen, self.sg_order)

            # ── Step 3: Median Filter Background ─────
            # หา background ของพื้นรอบๆ
            # window ใหญ่ = background เรียบ
            background = median_filter(smooth, size=self.med_window, mode='nearest')

            # ── Step 4: Flatten Signal ────────────────
            # background - smooth = peak ที่นูนขึ้น
            # รอยเชื่อม = range น้อยกว่า background
            # → flattened signal สูงขึ้นตรงรอยเชื่อม
            flattened = background - smooth

            # ── Step 5: Lateral Correction ───────────
            # วัด avg ของ ROI เปรียบกับ center
            # → บอกว่าหุ่นเยื้องซ้าย/ขวาเท่าไหร่
            roi_avg       = float(np.mean(raw))
            lateral_error = roi_avg - self.center_avg

            if abs(lateral_error) > self.lateral_deadband:
                lateral_angle = lateral_error * self.lateral_scale
            else:
                lateral_angle = 0.0

            # ── Step 6: Find Peaks ────────────────────
            # หา peak ที่โดดเด่นพอ
            peaks, props = find_peaks(
                flattened,
                prominence=self.min_prominence,
                width=0
            )

            found_weld = False
            best_angle = float('nan')

            if len(peaks) > 0:
                prominences = props['prominences']
                widths      = props['widths']

                # เรียง peak จากโดดเด่นมากสุดไปน้อยสุด
                sorted_idx = np.argsort(prominences)[::-1]

                # ── Step 7: T-Junction Detection ─────
                # ถ้าเจอ 2 peaks ใหญ่ใกล้กัน = T-junction
                # หยุดหุ่น
                if len(sorted_idx) >= 2:
                    idx1 = sorted_idx[0]
                    idx2 = sorted_idx[1]
                    top1 = float(prominences[idx1])
                    top2 = float(prominences[idx2])
                    sep  = abs(int(peaks[idx1]) - int(peaks[idx2]))

                    ratio_valid = (
                        top1 > 0.0 and
                        top2 >= self.t_junction_ratio * top1
                    )
                    sep_valid  = sep >= self.t_junction_min_separation
                    prom_valid = (
                        top1 >= self.t_junction_min_prominence and
                        top2 >= self.t_junction_min_prominence
                    )

                    self.get_logger().warn(
                        f'T1={top1:.4f} | T2={top2:.4f} | '
                        f'Ratio={top2/top1:.2f} | Sep={sep}'
                    )

                    if ratio_valid and sep_valid and prom_valid:
                        self.t_junction_count += 1
                        self.get_logger().warn(
                            f'Possible T-junction '
                            f'({self.t_junction_count}/'
                            f'{self.t_junction_confirm_threshold})'
                        )
                        if self.t_junction_count >= self.t_junction_confirm_threshold:
                            self.system_stopped = True
                            self.publish_nan(
                                'T-junction confirmed → STOP',
                                status='T_JUNCTION'
                            )
                            return
                    else:
                        self.t_junction_count = 0
                else:
                    self.t_junction_count = 0

                # ── Step 8: Normal Weld Detection ────
                # เลือก peak ที่ดีที่สุดจาก top 3
                # เช็ค width + angle_diff + height
                for k in range(min(len(sorted_idx), 3)):
                    idx           = sorted_idx[k]
                    local_idx     = int(peaks[idx])
                    current_width  = float(widths[idx])
                    current_height = float(flattened[local_idx])
                    global_idx    = self.roi_start + local_idx

                    current_angle = self.index_to_angle(
                        global_idx,
                        msg.angle_min,
                        msg.angle_increment
                    )

                    loc_valid    = self.is_valid_weld(current_angle, self.last_valid_angle)
                    height_valid = current_height >= self.min_height_threshold

                    if current_width <= self.max_width and loc_valid and height_valid:
                        found_weld            = True
                        best_angle            = current_angle
                        self.last_valid_angle = best_angle
                        self.missed_count     = 0
                        self.t_junction_count = 0
                        break

            # ── Step 9: รวม heading + lateral ────────
            if found_weld and not math.isnan(best_angle):

                combined = best_angle + lateral_angle

                # Median smooth 5 frames กัน noise
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

                self.publish_status('WELD_FOUND')
                out      = Float32()
                out.data = float(smoothed)
                self.angle_pub.publish(out)

            else:
                # ── Step 10: No Weld ─────────────────
                self.angle_history = []
                self.missed_count += 1
                if self.missed_count >= self.reset_threshold:
                    self.last_valid_angle = float('nan')
                    self.missed_count     = 0
                self.publish_nan('No valid weld', status='NO_WELD')

        except Exception as e:
            self.last_valid_angle = float('nan')
            self.t_junction_count = 0
            self.angle_history    = []
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
