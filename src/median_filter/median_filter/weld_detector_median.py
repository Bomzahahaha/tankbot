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

        # ══════════════════════════════════════════
        # ROI — ใช้ของรุ่นพี่ (กว้างพอสำหรับ scan)
        # ปรับ roi_start/roi_end ให้ตรงกับ
        # ตำแหน่งเส้นจริงบนถัง
        # ══════════════════════════════════════════
        self.roi_start = 313
        self.roi_end   = 421   # 109 steps

        # ══════════════════════════════════════════
        # FILTER
        # Savitzky-Golay: smooth noise รักษา peak
        # sg_framelen ต้องคี่ และ < roi size
        # Median Filter: หา background ของพื้น
        # med_window ต้องน้อยกว่า roi size
        # ══════════════════════════════════════════
        self.sg_order    = 3
        self.sg_framelen = 11   # smooth 5 จุดซ้าย-ขวา
        self.med_window  = 61   # background window

        # ══════════════════════════════════════════
        # PEAK PARAMETERS
        # min_prominence: peak ต้องโดดเด่นเท่าไหร่
        # min_height:     peak ต้องสูงเท่าไหร่
        # max_width:      peak กว้างได้ไม่เกินนี้
        # ══════════════════════════════════════════
        self.min_prominence       = 0.005
        self.min_height_threshold = 0.0035
        self.max_width            = 40

        # ══════════════════════════════════════════
        # TRACKING
        # angle_diff_threshold: กัน angle กระโดด
        # reset_threshold: miss กี่ frame ค่อย reset
        # last_known_angle: มุม confirm ล่าสุด ใช้เทียบกัน
        #   single-frame กระโดด เคลียร์ทิ้งเป็น nan เมื่อหลุด
        #   นานพอ (reset_threshold/timeout/error) เพื่อเปิด
        #   gate ใหม่ให้ล็อกใหม่ได้อิสระ (ไม่ค้างเป็นค่าเก่าตลอดไป)
        # ══════════════════════════════════════════
        self.last_valid_angle     = float('nan')
        self.last_known_angle     = float('nan')
        self.missed_count         = 0
        self.reset_threshold      = 10
        self.angle_diff_threshold = math.radians(2.72)  # เข้มกว่าเดิมมาก

        # ══════════════════════════════════════════
        # RE-LOCK CONFIRMATION GATE
        # ป้องกันการ "ล็อกผิดจุด" ตอนเริ่มต้น หรือตอน
        # กลับมาจับใหม่หลัง NO_WELD — เดิมพอ
        # last_known_angle เป็น nan ระบบจะเชื่อ peak
        # แรกที่เจอทันที ถ้า peak นั้นไม่ใช่รอยเชื่อมจริง
        # (เช่น noise/ขอบชิ้นงาน) จะล็อกผิดค้างไปยาว
        # เพราะ angle_diff_threshold กันไม่ให้กลับไปหา
        # ตัวจริงได้อีก ตอนนี้ต้องเห็นมุมนิ่งซ้ำกัน
        # relock_confirm_threshold เฟรมก่อน ถึงจะยอมล็อกจริง
        # ══════════════════════════════════════════
        self.relock_candidate_angle   = float('nan')
        self.relock_candidate_count   = 0
        self.relock_confirm_threshold = 4
        self.relock_tolerance         = math.radians(3.5)

        # ══════════════════════════════════════════
        # COAST — ประคองต่อด้วยมุมล่าสุดตอนหลุดสั้นๆ
        # กันอาการกระตุกจากการหลุดแว้บเดียว (noise) —
        # ไม่ต้อง publish NO_WELD ทันทีตั้งแต่เฟรมแรก
        # ที่หลุด ถ้ายังไม่หลุดนานเกิน coast_max เฟรม
        # ให้เชื่อ last_valid_angle ไปพลางก่อน หุ่นเดินช้า
        # มาก (max_linear_speed=0.035 m/s) ความเสี่ยง
        # จากการ coast ผิดจึงต่ำมาก
        # ══════════════════════════════════════════
        self.coast_count = 0
        self.coast_max    = 0   # หลุดได้กี่เฟรมก่อนยอมแพ้ (ปรับได้)

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
        self.lateral_scale    = 0.0
        self.lateral_deadband = 0.004
        self.heading_offset   = math.radians(0.0)

        # ══════════════════════════════════════════
        # TIMEOUT
        # ══════════════════════════════════════════
        self.last_scan_time    = self.get_clock().now()
        self.scan_timeout_sec  = 3.0
        self.timer             = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered = False

        self.get_logger().info(
            'Weld Detector (Senior + Lateral + Re-lock Gate + Coast) Started.'
        )

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

    def reset_relock_gate(self):
        # เรียกทุกครั้งที่ last_known_angle ถูกล้างเป็น nan
        # เพื่อให้รอบต่อไปเริ่มยืนยันมุมใหม่ตั้งแต่ศูนย์
        # ไม่ใช้ candidate เก่าที่อาจค้างมาจากนานแล้ว
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

    # ════════════════════════════════════════════════
    # MAIN CALLBACK
    # ════════════════════════════════════════════════
    def scan_callback(self, msg: LaserScan):

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

            # เก็บสถานะว่า "ก่อนเฟรมนี้" ล็อกอยู่หรือยัง
            # ใช้ตัดสินว่าต้องผ่าน re-lock gate ไหม
            was_locked = not math.isnan(self.last_known_angle)

            if len(peaks) > 0:
                prominences = props['prominences']
                widths      = props['widths']

                # เรียง peak จากโดดเด่นมากสุดไปน้อยสุด
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
                        best_angle = min(candidates, key=lambda a: abs(a))
                    else:
                        best_angle = min(candidates, key=lambda a: abs(a - self.last_known_angle))
                    found_weld            = True
                    self.last_valid_angle = best_angle
                    self.last_known_angle = best_angle
                    self.missed_count     = 0
                    self.coast_count       = 0
                    raw_msg = Float32()
                    raw_msg.data = float(best_angle)
                    self.raw_angle_pub.publish(raw_msg)

                # ── Step 7: เลือก peak ที่ดีที่สุดจาก top 3 ─
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

                    loc_valid    = self.is_valid_weld(current_angle)
                    height_valid = current_height >= self.min_height_threshold

                    if current_width <= self.max_width and loc_valid and height_valid:
                        found_weld            = True
                        best_angle            = current_angle
                        self.last_valid_angle = best_angle
                        self.last_known_angle = best_angle
                        self.missed_count     = 0
                        self.coast_count       = 0
                        raw_msg = Float32()
                        raw_msg.data  = float(current_angle)
                        self.raw_angle_pub.publish(raw_msg)
                        break

            # ── Step 7.5: Re-lock Confirmation Gate ──
            # ถ้าเฟรมนี้เพิ่งจะ "ล็อกใหม่" (ก่อนหน้านี้ยังไม่
            # ล็อกอยู่ / last_known_angle เป็น nan) ห้ามเชื่อ
            # ทันทีจาก peak เดียว ต้องเห็นมุมนิ่งใกล้เคียงกัน
            # ซ้ำ relock_confirm_threshold เฟรมก่อน ถึงจะปล่อย
            # ให้ใช้งานจริง กันล็อกผิดจุดตอนเริ่มต้น/กลับมาใหม่
            if found_weld and not was_locked:
                if math.isnan(self.relock_candidate_angle):
                    self.relock_candidate_angle = best_angle
                    self.relock_candidate_count = 1
                elif abs(best_angle - self.relock_candidate_angle) < self.relock_tolerance:
                    self.relock_candidate_count += 1
                else:
                    # มุมกระโดด แปลว่ายังไม่นิ่ง เริ่มนับใหม่
                    self.relock_candidate_angle = best_angle
                    self.relock_candidate_count = 1

                if self.relock_candidate_count < self.relock_confirm_threshold:
                    # ยังยืนยันไม่พอ ยกเลิกการล็อกที่เพิ่งเกิด
                    # เฟรมนี้ ให้นับเป็น "ยังไม่เจอ" ไปก่อน
                    self.last_valid_angle = float('nan')
                    self.last_known_angle = float('nan')
                    found_weld            = False
                    best_angle            = float('nan')
                else:
                    # ยืนยันครบแล้ว ปลดล็อกให้ใช้งานได้จริง
                    self.reset_relock_gate()

            # ── Step 8: รวม heading + lateral ────────
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

                corrected = smoothed - self.heading_offset
                self.publish_status('WELD_FOUND')
                out      = Float32()
                out.data = float(corrected)
                self.angle_pub.publish(out)

            else:
                # ── Step 9: No Weld / Coast ──────────
                self.angle_history = []
                self.missed_count += 1

                if self.missed_count >= self.reset_threshold:
                    # หลุดนานเกินไปจริง ยอมรับว่า NO_WELD
                    # และเปิด gate ใหม่ให้ล็อกได้อิสระรอบหน้า
                    self.last_valid_angle  = float('nan')
                    self.last_known_angle  = float('nan')
                    self.missed_count      = 0
                    self.coast_count       = 0
                    self.reset_relock_gate()
                    self.publish_nan('No valid weld', status='NO_WELD')
                elif (
                    self.coast_count < self.coast_max
                    and not math.isnan(self.last_valid_angle)
                ):
                    # หลุดสั้นๆ ยังไม่เกิน coast_max เฟรม
                    # ประคองต่อด้วยมุมล่าสุด ไม่เพิ่ง publish
                    # NO_WELD ทันที กันอาการกระตุกจาก noise
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
                    # coast หมดแล้วแต่ยังไม่ถึง reset_threshold
                    # เต็ม ก็ publish NO_WELD ตามปกติไปก่อน
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
