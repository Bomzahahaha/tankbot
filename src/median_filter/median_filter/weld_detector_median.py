"""
weld_detector_median — หาตำแหน่ง (มุม) ของรอยเชื่อมจากข้อมูล LiDAR

Subscribe : /scan          (LaserScan)
Publish   : /best_angle    (Float32, rad) มุมรอยเชื่อมที่ยืนยันแล้ว -> เข้า pid_node
            /raw_angle     (Float32, rad) มุมดิบก่อนกรอง (ใช้ debug/log)
            /weld_status   (String) WELD_FOUND / NO_WELD / TIMEOUT / ERROR / T_JUNCTION

มี 3 detection_mode สลับได้ด้วยตัวแปรเดียว (บรรทัด ~35):
  'no_filter'     : ไม่กรองอะไรเลย ใช้จุดระยะสั้นสุดตรงๆ — เก็บไว้เทียบ baseline เท่านั้น
  'senior_filter' : อัลกอริทึมของรุ่นพี่ (T-junction + simple gate) — เก็บไว้เทียบผลเท่านั้น
  'my_filter'     : โหมดที่ใช้งานจริงปัจจุบัน (shadow-candidate + relock + drift gate)

หมายเหตุการอ่าน comment พารามิเตอร์:
  🔧 = ปรับ/เปลี่ยนค่าจริงหลายครั้งจากการทดสอบ (มีหลักฐานใน git history)
  ⚪ = ตั้งค่าครั้งเดียวตั้งแต่แรก ไม่เคยกลับมาปรับอีกเลยตลอดโปรเจกต์
"""
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
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.angle_pub     = self.create_publisher(Float32, '/best_angle', 10)
        self.status_pub    = self.create_publisher(String,  '/weld_status', 10)
        self.raw_angle_pub = self.create_publisher(Float32, '/raw_angle', 10)

        # --- สลับโหมดตรวจจับ: เปลี่ยนบรรทัดนี้บรรทัดเดียว (ดูคำอธิบาย 3 โหมดด้านบนไฟล์) ---
        self.detection_mode = 'my_filter'   # 🔧 สลับไปมาระหว่าง 3 โหมดตลอดโปรเจกต์เพื่อเทียบผล

        # ── ROI: แต่ละโหมดใช้ขอบเขตของตัวเอง (เทียบทั้งระบบ ไม่ใช่แค่ algorithm ล้วนๆ) ──
        self.roi_start_mine   = 370   # 🔧 ปรับ 3 ครั้ง (1→370→0→370) แคบลงเรื่อยๆ ตัด noise ข้างเคียง
        self.roi_end_mine     = 398   # 🔧 ปรับ 3 ครั้ง (600→398→725→398)
        self.roi_start_senior = 330   # ⚪ ของรุ่นพี่ ไม่เคยแตะ
        self.roi_end_senior   = 438   # ⚪ ของรุ่นพี่ ไม่เคยแตะ

        # ── พารามิเตอร์กรองสัญญาณ (ใช้ร่วมกันทั้ง senior_filter และ my_filter) ──
        self.sg_order    = 3    # ⚪ ไม่เคยปรับเลยตลอดโปรเจกต์ (ดีกรีพหุนาม Savitzky-Golay)
        self.sg_framelen = 9    # 🔧 ปรับ 5 ครั้ง (15→21→7→11→9) ยิ่งมาก = เรียบขึ้นแต่ช้าตอบสนอง
        self.med_window  = 21   # 🔧 ปรับมากที่สุดในกลุ่มนี้ 6 ครั้ง (51→71→11→61→81→21)

        self.min_prominence       = 0.0015  # 🔧 ปรับ 5 ครั้ง (0.002→0.001→0.003→0.0015→0.005→0.0015)
        self.min_height_threshold = 0.0035  # 🔧 ปรับ 3 ครั้ง (0.004→0.0035→0.005→0.0035)
        self.max_width             = 40      # 🔧 ปรับ 4 ครั้ง (60→100→20→40)

        self.heading_offset = math.radians(0.0)   # 🔧 ปรับบ่อยมาก (8 ครั้ง) ตามการติดตั้งเซนเซอร์แต่ละรอบ ไม่ใช่ tuning เชิง algorithm

        self.last_scan_time    = self.get_clock().now()
        self.scan_timeout_sec  = 3.0   # ⚪ ไม่เคยปรับ
        self.timer              = self.create_timer(0.5, self.check_scan_timeout)
        self.timeout_triggered  = False

        # =====================================================================
        # พารามิเตอร์เฉพาะโหมด 'my_filter' (โหมดที่ใช้งานจริง)
        # =====================================================================
        self.last_valid_angle = float('nan')
        self.last_known_angle = float('nan')
        self.missed_count     = 0
        self.reset_threshold  = 10   # 🔧 ปรับ 1 ครั้ง (5 → 10) เฟรมไม่เจอติดกันก่อนลืมตำแหน่งเดิมทั้งหมด
        self.angle_diff_threshold = math.radians(1.0)   # 🔧 ตัวที่ถูกปรับบ่อยที่สุดในไฟล์นี้ (12 ครั้ง! 3°→...→1°→1.5°)

        # Relock gate: ต้องเห็นตำแหน่งเดิมซ้ำกี่ครั้งก่อนเชื่อว่า "เจอกลับมาแล้วจริง"
        self.relock_candidate_angle   = float('nan')
        self.relock_candidate_count   = 0
        self.relock_confirm_threshold = 5    # 🔧 ปรับ 3 ครั้ง (5→4→3→5)
        self.relock_tolerance         = math.radians(2.0)   # 🔧 ปรับ 4 ครั้ง (2°→3°→3.5°→5°→2°)

        # Streak/drift gate: จับ "ค่อยๆ เบี่ยงทิศทางเดียวติดกันนาน" ที่ไม่ใช่การกระโดดฉับพลัน
        self.streak_sign = 0
        self.streak_len  = 0
        self.consistency_confirm_threshold = 6   # ⚪ ไม่เคยปรับเลย
        self.sign_eps = math.radians(0.05)       # ⚪ ไม่เคยปรับเลย

        # Shadow-candidate: ตอนยังไม่เคยล็อกเลย ใช้เสียงข้างมากจาก buffer แทนเชื่อจุดเดียว
        self.has_ever_locked = False
        self.candidate_buffer = []
        self.buffer_size = 20                    # ⚪ ไม่เคยปรับเลย (เฟรม ~2.0s ที่ 10Hz)
        self.mode_bin_width = math.radians(2.0)   # ⚪ ไม่เคยปรับเลย
        self.mode_confirm_ratio = 0.5             # 🔧 ปรับ 1 ครั้ง (0.4 → 0.5)

        # ⚪🔴 DEAD CODE: coast_max = 0 -> เงื่อนไข coast_count < coast_max ไม่มีทางเป็นจริง
        #    กลไก "เดาต่อจากค่าล่าสุดชั่วคราวตอนเพิ่งหลุด" นี้จึงไม่ทำงานจริงในปัจจุบัน
        #    (⚪ ไม่เคยปรับเลย ตั้งเป็น 0 มาตั้งแต่แรก)
        self.coast_count = 0
        self.coast_max   = 0

        self.angle_history = []
        self.history_size  = 5   # ⚪ ไม่เคยปรับเลย (เฟรม median smoothing ท้ายสุดก่อน publish)

        # ⚪🔴 DEAD CODE: lateral_scale = 0.0 -> lateral_angle คำนวณได้เสมอ 0
        #    🔧 น่าสังเกต: ตัวนี้เคยปรับจริง (3.5 → 0.0) คือเคยเปิดใช้งานแล้วภายหลังปิดไปเลย
        #    ไม่ใช่ dead code ที่ไม่เคยลองแบบ tank_mode — เคยลองแล้วตัดสินใจปิด
        self.center_avg       = 0.093   # ⚪ ไม่เคยปรับ (เกี่ยวข้องกับกลไกที่ปิดอยู่)
        self.lateral_scale    = 0.0     # 🔧 เคยเป็น 3.5 มาก่อน แล้วปิดเป็น 0.0
        self.lateral_deadband = 0.004   # ⚪ ไม่เคยปรับ

        # =====================================================================
        # พารามิเตอร์เฉพาะโหมด 'senior_filter' — ⚠️ ไม่ใช่โหมดที่ใช้งานอยู่ (เก็บไว้เทียบผลเท่านั้น)
        # ทั้งหมดในกลุ่มนี้ ⚪ ไม่เคยแตะเลย เป็นค่าดั้งเดิมของรุ่นพี่ 100%
        # =====================================================================
        self.senior_last_valid_angle     = float('nan')
        self.senior_missed_count         = 0
        self.senior_reset_threshold      = 5
        self.senior_angle_diff_threshold = math.radians(3.0)

        self.senior_t_junction_ratio             = 0.60
        self.senior_t_junction_count             = 0
        self.senior_t_junction_confirm_threshold = 3
        self.senior_t_junction_min_separation    = 8
        self.senior_t_junction_min_prominence    = 0.001

        self.senior_system_stopped = False   # ตาม design เดิม: เจอ T-junction แล้วหยุดตลอดไป ไม่ฟื้นเอง

        self.get_logger().info(
            f"Weld Detector Started | detection_mode = '{self.detection_mode}'"
        )

    # =========================================================================
    def publish_status(self, status):
        msg = String(); msg.data = status
        self.status_pub.publish(msg)

    def publish_nan(self, reason='', status='NO_WELD'):
        out = Float32(); out.data = float('nan')
        self.angle_pub.publish(out)
        self.publish_status(status)
        if reason:
            self.get_logger().warn(reason)

    def reset_relock_gate(self):
        self.relock_candidate_angle = float('nan')
        self.relock_candidate_count = 0

    def check_scan_timeout(self):
        """เรียกทุก 0.5s — ถ้าไม่มี scan ใหม่นานเกิน scan_timeout_sec ให้เคลียร์ state ทั้งหมด"""
        dt = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if dt > self.scan_timeout_sec and not self.timeout_triggered:
            self.last_valid_angle = float('nan')
            self.last_known_angle = float('nan')
            self.missed_count     = 0
            self.coast_count      = 0
            self.reset_relock_gate()
            self.senior_last_valid_angle = float('nan')
            self.senior_missed_count     = 0
            self.senior_t_junction_count = 0
            self.timeout_triggered = True
            self.publish_nan(f'No scan {dt:.1f}s', status='TIMEOUT')

    def index_to_angle(self, index, angle_min, angle_increment):
        return angle_min + index * angle_increment

    def is_valid_weld(self, current_angle):
        """my_filter: มุมนี้ใกล้ตำแหน่งล่าสุดพอไหม (ถ้ายังไม่เคยล็อก = ผ่านเสมอ)"""
        if math.isnan(self.last_known_angle):
            return True
        return abs(current_angle - self.last_known_angle) < self.angle_diff_threshold

    def senior_is_valid_weld(self, current_angle, past_angle):
        """senior_filter: เกณฑ์เดียวกันแต่ threshold กว้างกว่า (3° vs 1°)"""
        if math.isnan(past_angle):
            return True
        return abs(current_angle - past_angle) < self.senior_angle_diff_threshold

    def get_mode_candidate(self):
        """หาตำแหน่งที่ปรากฏบ่อยที่สุดใน candidate_buffer (ใช้ตอน shadow-candidate lock ครั้งแรก)"""
        if len(self.candidate_buffer) < self.buffer_size:
            return None
        best_center, best_count = None, 0
        for center in self.candidate_buffer:
            count = sum(1 for v in self.candidate_buffer if abs(v - center) < self.mode_bin_width)
            if count > best_count:
                best_count, best_center = count, center
        if best_count / len(self.candidate_buffer) >= self.mode_confirm_ratio:
            cluster = [v for v in self.candidate_buffer if abs(v - best_center) < self.mode_bin_width]
            return sum(cluster) / len(cluster)
        return None

    # =========================================================================
    def scan_callback(self, msg: LaserScan):
        self.last_scan_time    = self.get_clock().now()
        self.timeout_triggered = False

        # เลือก ROI ตามโหมดที่ใช้งาน
        if self.detection_mode == 'senior_filter':
            roi_start, roi_end = self.roi_start_senior, self.roi_end_senior
        else:
            roi_start, roi_end = self.roi_start_mine, self.roi_end_mine

        raw = np.array(msg.ranges[roi_start:roi_end + 1], dtype=float)
        raw[np.isinf(raw)] = msg.range_max
        raw[np.isnan(raw)] = 0.0

        if len(raw) < 10:
            self.publish_nan('ROI too short', status='ERROR')
            return

        # =================================================================
        # โหมด 1: NO_FILTER — จุดระยะสั้นสุด (ลึกสุด) ตรงๆ ไม่ผ่านการกรองใดๆ
        # ⚠️ ไม่ใช่โหมดที่ใช้งานอยู่ — เก็บไว้เทียบ baseline เท่านั้น
        # =================================================================
        if self.detection_mode == 'no_filter':
            deepest_local_idx = int(np.argmin(raw))
            global_idx = roi_start + deepest_local_idx
            raw_best  = self.index_to_angle(global_idx, msg.angle_min, msg.angle_increment)
            corrected = raw_best - self.heading_offset
            self.publish_status('WELD_FOUND')
            out = Float32(); out.data = float(corrected)
            self.angle_pub.publish(out)
            raw_msg = Float32(); raw_msg.data = float(raw_best)
            self.raw_angle_pub.publish(raw_msg)
            self.get_logger().info(f'[NO FILTER] angle={math.degrees(corrected):.2f} deg')
            return

        # =================================================================
        # โหมด 2: SENIOR_FILTER — อัลกอริทึมของรุ่นพี่ (T-junction detect + simple gate)
        # ⚠️ ไม่ใช่โหมดที่ใช้งานอยู่ — เก็บไว้เทียบผลเท่านั้น
        # =================================================================
        if self.detection_mode == 'senior_filter':
            if self.senior_system_stopped:
                return   # เจอ T-junction แล้วหยุดตลอดไปตาม design เดิม ไม่ฟื้นเอง

            try:
                smooth     = savgol_filter(raw, self.sg_framelen, self.sg_order)
                background = median_filter(smooth, size=self.med_window, mode='nearest')
                flattened  = background - smooth

                peaks, props = find_peaks(flattened, prominence=self.min_prominence, width=0)

                found_weld = False
                best_angle = float('nan')

                if len(peaks) > 0:
                    prominences = props['prominences']
                    widths      = props['widths']
                    sorted_idx  = np.argsort(prominences)[::-1]

                    # --- T-junction detection: 2 peak สูงใกล้กันและห่างกันพอ = สงสัยเป็นจุดตัด ไม่ใช่รอยเชื่อมเดี่ยว ---
                    if len(sorted_idx) >= 2:
                        idx1, idx2   = sorted_idx[0], sorted_idx[1]
                        top1, top2   = prominences[idx1], prominences[idx2]
                        peak1, peak2 = peaks[idx1], peaks[idx2]
                        separation   = abs(int(peak1) - int(peak2))

                        ratio_valid       = top1 > 0.0 and top2 > (self.senior_t_junction_ratio * top1)
                        separation_valid  = separation >= self.senior_t_junction_min_separation
                        prominence_valid  = (top1 >= self.senior_t_junction_min_prominence and
                                              top2 >= self.senior_t_junction_min_prominence)

                        if ratio_valid and separation_valid and prominence_valid:
                            self.senior_t_junction_count += 1
                            if self.senior_t_junction_count >= self.senior_t_junction_confirm_threshold:
                                self.senior_last_valid_angle = float('nan')
                                self.senior_missed_count = 0
                                self.publish_nan('[SENIOR] Confirmed T-junction -> STOP PUBLISHING', status='T_JUNCTION')
                                self.senior_system_stopped = True
                                return
                        else:
                            self.senior_t_junction_count = 0
                    else:
                        self.senior_t_junction_count = 0

                    # --- ตรวจจับปกติ: ไล่ top-3 peak ใช้ตัวแรกที่ผ่านเกณฑ์ ---
                    num_candidates = min(len(sorted_idx), 3)
                    for k in range(num_candidates):
                        idx            = sorted_idx[k]
                        local_idx      = int(peaks[idx])
                        current_width  = float(widths[idx])
                        current_height = float(flattened[local_idx])
                        global_idx     = roi_start + local_idx
                        current_angle  = self.index_to_angle(global_idx, msg.angle_min, msg.angle_increment)

                        loc_valid    = self.senior_is_valid_weld(current_angle, self.senior_last_valid_angle)
                        height_valid = current_height >= self.min_height_threshold

                        if current_width <= self.max_width and loc_valid and height_valid:
                            found_weld = True
                            best_angle = current_angle
                            self.senior_last_valid_angle = best_angle
                            self.senior_missed_count     = 0
                            self.senior_t_junction_count = 0
                            break

                if not found_weld:
                    self.senior_missed_count += 1
                    if self.senior_missed_count >= self.senior_reset_threshold:
                        self.senior_last_valid_angle = float('nan')
                        self.senior_missed_count = 0
                    self.publish_nan('[SENIOR] No valid weld', status='NO_WELD')
                    return

                corrected = best_angle - self.heading_offset
                self.publish_status('WELD_FOUND')
                out = Float32(); out.data = float(corrected)
                self.angle_pub.publish(out)
                raw_msg = Float32(); raw_msg.data = float(best_angle)
                self.raw_angle_pub.publish(raw_msg)
                self.get_logger().info(f'[SENIOR FILTER] angle={math.degrees(corrected):.2f} deg')

            except Exception as e:
                self.senior_last_valid_angle = float('nan')
                self.senior_t_junction_count = 0
                self.publish_nan(f'[SENIOR] Error: {e}', status='ERROR')
            return

        # =================================================================
        # โหมด 3: MY_FILTER — โหมดที่ใช้งานจริงปัจจุบัน
        # (shadow-candidate lock ครั้งแรก + relock gate + drift/streak gate)
        # =================================================================
        try:
            smooth     = savgol_filter(raw, self.sg_framelen, self.sg_order)
            background = median_filter(smooth, size=self.med_window, mode='nearest')
            flattened  = background - smooth

            # ⚠️ lateral_angle คำนวณได้เสมอ 0.0 เพราะ lateral_scale=0 (ดู DEAD CODE ด้านบน)
            roi_avg       = float(np.mean(raw))
            lateral_error = roi_avg - self.center_avg
            lateral_angle = lateral_error * self.lateral_scale if abs(lateral_error) > self.lateral_deadband else 0.0

            peaks, props = find_peaks(flattened, prominence=self.min_prominence, width=0)

            found_weld = False
            best_angle = float('nan')
            was_locked = not math.isnan(self.last_known_angle)

            if len(peaks) > 0:
                prominences = props['prominences']
                widths      = props['widths']
                sorted_idx  = np.argsort(prominences)[::-1]
                candidates  = []
                top1_angle  = float('nan')

                for k in range(min(len(sorted_idx), 3)):
                    idx            = sorted_idx[k]
                    local_idx      = int(peaks[idx])
                    current_width  = float(widths[idx])
                    current_height = float(flattened[local_idx])
                    global_idx     = roi_start + local_idx
                    current_angle  = self.index_to_angle(global_idx, msg.angle_min, msg.angle_increment)
                    loc_valid    = self.is_valid_weld(current_angle)
                    height_valid = current_height >= self.min_height_threshold

                    if k == 0 and current_width <= self.max_width and height_valid:
                        top1_angle = current_angle   # peak เด่นสุด เก็บเข้า buffer ไม่ว่าจะผ่าน loc_valid ไหม
                    if current_width <= self.max_width and loc_valid and height_valid:
                        candidates.append(current_angle)

                if not math.isnan(top1_angle):
                    self.candidate_buffer.append(top1_angle)
                    if len(self.candidate_buffer) > self.buffer_size:
                        self.candidate_buffer.pop(0)

                if candidates:
                    if math.isnan(self.last_known_angle):
                        # ยังไม่เคยล็อกตอนนี้ (เพิ่งเริ่ม หรือเพิ่งหลุด)
                        if not self.has_ever_locked:
                            # ครั้งแรกสุดของโปรแกรม: ยังไม่มี buffer อ้างอิง ใช้จุดใกล้ 0 องศาสุด
                            raw_best = min(candidates, key=lambda a: abs(a))
                        else:
                            # เคยล็อกมาก่อนแล้ว: ใช้เสียงข้างมากจาก buffer ช่วยเลือก (กัน noise เดี่ยวหลอก)
                            mode_candidate = self.get_mode_candidate()
                            if mode_candidate is not None:
                                raw_best = min(candidates, key=lambda a: abs(a - mode_candidate))
                            else:
                                raw_best = min(candidates, key=lambda a: abs(a))
                    else:
                        # กำลังล็อกอยู่: เลือกตัวใกล้ตำแหน่งล่าสุดที่สุด
                        raw_best = min(candidates, key=lambda a: abs(a - self.last_known_angle))

                    # --- Drift/streak gate: เบี่ยงทิศทางเดียวติดกันนานเกินไป = ไม่เชื่อ (ค้างค่าเดิมไว้ก่อน) ---
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
                        found_weld = True
                        self.last_valid_angle = best_angle
                        self.last_known_angle = best_angle
                        self.missed_count     = 0
                        self.coast_count      = 0
                        raw_msg = Float32(); raw_msg.data = float(best_angle)
                        self.raw_angle_pub.publish(raw_msg)
                    else:
                        best_angle = self.last_known_angle   # ไม่เชื่อค่าใหม่ ใช้ค่าเดิมไปก่อน
                        found_weld = True

            # --- Relock gate: เพิ่งกลับมาเจอหลังจากหลุด ต้องเห็นตำแหน่งเดิมซ้ำก่อนเชื่อ ---
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
                    # ยังไม่ครบจำนวนยืนยัน -> ยังไม่เชื่อ ปล่อยเป็น NO_WELD ต่อ
                    self.last_valid_angle = float('nan')
                    self.last_known_angle = float('nan')
                    found_weld = False
                    best_angle = float('nan')
                else:
                    self.has_ever_locked = True
                    self.reset_relock_gate()

            if found_weld and not math.isnan(best_angle):
                combined = best_angle + lateral_angle   # lateral_angle = 0 เสมอ (dead code)
                self.angle_history.append(combined)
                if len(self.angle_history) > self.history_size:
                    self.angle_history.pop(0)
                smoothed = float(np.median(self.angle_history))

                self.get_logger().info(
                    f'[MY FILTER] heading={math.degrees(best_angle):.2f} | out={math.degrees(smoothed):.2f} deg'
                )

                corrected = smoothed - self.heading_offset
                self.publish_status('WELD_FOUND')
                out = Float32(); out.data = float(corrected)
                self.angle_pub.publish(out)
            else:
                self.angle_history = []
                self.missed_count += 1
                if self.missed_count >= self.reset_threshold:
                    # ไม่เจอนานเกินไป -> ลืมตำแหน่งเดิมทั้งหมด กลับสู่สถานะเริ่มต้น
                    self.last_valid_angle = float('nan')
                    self.last_known_angle = float('nan')
                    self.missed_count     = 0
                    self.coast_count      = 0
                    self.streak_sign, self.streak_len = 0, 0
                    self.reset_relock_gate()
                    self.publish_nan('No valid weld', status='NO_WELD')
                elif self.coast_count < self.coast_max and not math.isnan(self.last_valid_angle):
                    # ⚠️ ไม่มีทางเข้าเงื่อนไขนี้จริง เพราะ coast_max=0 (dead code)
                    self.coast_count += 1
                    self.publish_status('WELD_FOUND')
                    out = Float32(); out.data = float(self.last_valid_angle)
                    self.angle_pub.publish(out)
                else:
                    self.publish_nan('No valid weld', status='NO_WELD')

        except Exception as e:
            self.last_valid_angle = float('nan')
            self.last_known_angle = float('nan')
            self.angle_history    = []
            self.coast_count      = 0
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
