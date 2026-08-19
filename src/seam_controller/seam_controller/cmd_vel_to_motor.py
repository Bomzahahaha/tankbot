import csv
import math
import os
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class CmdVelToMotorClosedLoop(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_motor")

        # --- กายภาพหุ่นยนต์ ---
        self.wheel_base   = 0.30
        self.wheel_radius = 0.0295   # Calibrated 29.5 mm
        self.ticks_per_rev = 200

        # =====================================================================
        # --- Feedforward: ตาราง Calibration 3 ชุด (floor / vertical / horizontal) ---
        # =====================================================================
        # ชุดที่ 1: พื้นราบ (floor) — วัดจากพื้นปกติ
        self.pwm_calib_pwm_floor   = [0.20, 0.40, 0.60, 0.80, 1.00]
        self.pwm_calib_speed_floor = [0.034, 0.080, 0.126, 0.175, 0.220]  # m/s

        # ชุดที่ 2: ถัง แนวดิ่ง (vertical) — วัดจากถังจริง ตอนไต่ขึ้น
        self.pwm_calib_pwm_vertical   = [0.20, 0.40, 0.60, 0.80, 1.00]
        self.pwm_calib_speed_vertical = [0.000, 0.000, 0.0385, 0.0817, 0.0871]  # m/s
        # 20%,40% = dead zone จริง (ไม่ขยับเลยบนแนวดิ่ง)

        # ชุดที่ 3: ถัง แนวนอน (horizontal) — วัดจากถังจริง ตอนเดินรอบวง
        self.pwm_calib_pwm_horizontal   = [0.20, 0.40, 0.60, 0.80, 1.00]
        self.pwm_calib_speed_horizontal = [0.0000, 0.0672, 0.1070, 0.1606, 0.2471]  # m/s
        # เร็วกว่าแนวดิ่งมาก (ไม่มีแรงโน้มถ่วงต้านตามทิศทางเดิน)

        # --- สวิตช์เลือกพื้นผิว — เปลี่ยนแค่บรรทัดนี้บรรทัดเดียวตอนย้ายไปทดสอบพื้นผิวอื่น ---
        # ตัวเลือก: 'floor' | 'vertical' | 'horizontal'
        self.surface_mode = 'horizontal'

        self.max_spd_r = 0.22
        self.max_spd_l = 0.22

        # --- PID แยกฝั่ง (Independent) ---
        self.kp_r = 3.0;  self.ki_r = 0.35
        self.kp_l = 3.5;  self.ki_l = 0.40

        self.integ_limit = 0.5
        self.speed_dt    = 0.1
        self.alpha       = 0.6
        self.cmd_timeout = 0.5

        # --- Kick-start (ยืนยันแล้วจากทดสอบจริงบนถัง: pwm=0.6, duration=0.3 -> ขยับใน 0.047s) ---
        self.kickstart_pwm      = 0.00
        self.kickstart_duration = 0.3   # วินาที
        self.kickstart_movement_threshold = 0.002  # m/s

        # --- Grace period (กัน kick-start ซ้ำถี่ๆ ตอน WELD_FOUND/NO_WELD flicker) ---
        self.kickstart_grace_period = 0.5   # วินาที
        self.last_moving_time_r = None
        self.last_moving_time_l = None

        self.kickstart_start_r = None
        self.kickstart_start_l = None

        # --- Holding torque (กันไถลตอน NO_WELD บนพื้นผิวแนวดิ่ง) ---
        # หมายเหตุ: แนวนอนไม่มีแรงโน้มถ่วงต้านตามทิศทางเดินโดยตรง อาจไม่จำเป็นเท่าแนวดิ่ง
        # แต่เปิดไว้เป็นค่าเริ่มต้นเผื่อความปลอดภัย ปิดได้ด้วย holding_enabled=False ถ้าทดสอบแล้วไม่จำเป็น
        self.holding_pwm = 0.0
        self.holding_enabled = True
        self.holding_max_duration = 5.0   # วินาที — กันมอเตอร์ร้อนสะสม (สงสัยเกี่ยวกับ thermal/undervoltage)
        self.holding_start_time_r = None
        self.holding_start_time_l = None

        # --- Manual PWM test mode (สำหรับเก็บตาราง calibration พื้นผิวใหม่ๆ ในอนาคต) ---
        self.manual_pwm_mode  = False
        self.manual_pwm_value = 0.0

        # --- ตัวแปรภายใน ---
        self.last_cmd_time = time.time()
        self.target_r = self.target_l = 0.0
        self.ticks_r  = self.ticks_l  = 0
        self.dist_r   = self.dist_l   = 0.0
        self.offset_r = self.offset_l = None

        self.pt_r = self.ptime_r = None; self.speed_r = 0.0; self.integ_r = 0.0
        self.pt_l = self.ptime_l = None; self.speed_l = 0.0; self.integ_l = 0.0

        self.t0 = time.time()
        self.log_counter = 0
        self.cmd_vx = self.cmd_wz = 0.0
        self.best_angle  = float("nan")
        self.raw_angle  = float("nan")
        self.weld_status = "UNKNOWN"

        # --- Logger ---
        os.makedirs("logs", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.f = open(f"logs/log_{ts}.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t","vx","wz","tgt_r","tgt_l",
                         "spd_r","spd_l","err_r","err_l",
                         "ticks_r","ticks_l","dist_r","dist_l",
                         "raw_angle","filtered_angle","error","weld",
                         "ff_r","ff_l","kickstart_r","kickstart_l",
                         "holding_r","holding_l","manual_pwm_mode","surface_mode"])

        # --- Hardware ---
        self.lpwm = PWMOutputDevice(18, frequency=1000, initial_value=0.0)
        self.ldir = DigitalOutputDevice(17, initial_value=False)
        self.rpwm = PWMOutputDevice(19, frequency=1000, initial_value=0.0)
        self.rdir = DigitalOutputDevice(26, initial_value=False)

        self.ser = None
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
            time.sleep(2.0)
            self.get_logger().info("✅ Arduino Connected | Dual-PID Mode")
        except Exception as e:
            self.get_logger().warn(f"❌ Serial: {e}")

        self.create_subscription(Twist,  "/cmd_vel",     self.on_cmd,   10)
        self.create_subscription(Float32,"/best_angle",  self.on_angle, 10)
        self.create_subscription(Float32,"/raw_angle",  self.on_raw_angle, 10)
        self.create_subscription(String, "/weld_status", self.on_weld,  10)
        self.create_subscription(Float32, "/manual_pwm_test", self.on_manual_pwm, 10)
        self.pub_sr = self.create_publisher(Float32, "/right_wheel_speed", 10)
        self.pub_sl = self.create_publisher(Float32, "/left_wheel_speed",  10)
        self.pub_ticks_r = self.create_publisher(Float32, "/right_ticks", 10)
        self.pub_ticks_l = self.create_publisher(Float32, "/left_ticks", 10)

        self.create_timer(0.01, self.read_serial)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.05, self.log_data)
        self.get_logger().info(
            f"🚀 TankBot Dual-PID Node Started | surface_mode = '{self.surface_mode}' "
            "(kick-start + grace-period + holding-torque + 3-way calibration + manual-pwm-test)"
        )

    # =====================================================================
    def read_serial(self):
        if not self.ser: return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if "," not in line: continue
                for p in line.split(","):
                    p = p.strip()
                    if p.startswith("R:"):
                        raw = int(p[2:])
                        if self.offset_r is None: self.offset_r = raw
                        self.ticks_r = raw - self.offset_r
                    elif p.startswith("L:"):
                        raw = int(p[2:])
                        if self.offset_l is None: self.offset_l = raw
                        self.ticks_l = raw - self.offset_l
        except Exception as e:
            self.get_logger().warn(f"Serial: {e}")

    # =====================================================================
    def on_cmd(self, msg):
        self.manual_pwm_mode = False   # /cmd_vel ตัวจริงมา -> ปิดโหมด manual test ทันที
        self.last_cmd_time = time.time()
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        self.target_r = msg.linear.x + msg.angular.z * self.wheel_base / 2.0
        self.target_l = msg.linear.x - msg.angular.z * self.wheel_base / 2.0

        if abs(self.target_r) < 1e-4:
            self.integ_r = 0
        if abs(self.target_l) < 1e-4:
            self.integ_l = 0

    def on_angle(self, msg): self.best_angle  = msg.data
    def on_raw_angle(self, msg): self.raw_angle  = msg.data
    def on_weld (self, msg): self.weld_status = msg.data

    def on_manual_pwm(self, msg):
        """รับ PWM ตรงๆ (0.0-1.0) ข้าม PID/feedforward/kickstart/holding ทั้งหมด
        ใช้เก็บตาราง calibration พื้นผิวใหม่ๆ เท่านั้น"""
        self.manual_pwm_mode  = True
        self.manual_pwm_value = max(0.0, min(msg.data, 1.0))
        self.last_cmd_time = time.time()

    # =====================================================================
    def calc_speed(self, ticks, pt, ptime, spd):
        now = time.time()
        if pt is None: return 0.0, ticks, now
        dt = now - ptime
        if dt < self.speed_dt: return spd, pt, ptime
        delta = ticks - pt

        max_ticks = (0.22 / (2.0 * math.pi * self.wheel_radius)) \
                    * self.ticks_per_rev * dt * 2.0
        if abs(delta) > max_ticks:
            return spd * 0.8, ticks, now

        if delta == 0:
            spd = spd * 0.8
            if abs(spd) < 0.001: spd = 0.0
            return spd, ticks, now

        dist = (delta / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius
        raw  = dist / dt
        spd  = self.alpha * spd + (1.0 - self.alpha) * raw
        return spd, ticks, now

    # =====================================================================
    def get_active_calib_table(self):
        """เลือกตาราง calibration ตาม surface_mode ปัจจุบัน"""
        if self.surface_mode == 'floor':
            return self.pwm_calib_speed_floor, self.pwm_calib_pwm_floor
        elif self.surface_mode == 'vertical':
            return self.pwm_calib_speed_vertical, self.pwm_calib_pwm_vertical
        elif self.surface_mode == 'horizontal':
            return self.pwm_calib_speed_horizontal, self.pwm_calib_pwm_horizontal
        else:
            self.get_logger().warn(f"Unknown surface_mode '{self.surface_mode}', fallback to vertical")
            return self.pwm_calib_speed_vertical, self.pwm_calib_pwm_vertical

    def speed_to_pwm(self, target_speed):
        """หา PWM (0.0-1.0) จากความเร็วเป้าหมาย โดยเลือกตารางตาม surface_mode
        ตัดจุดที่ speed=0 ออกจากการ interpolate (เช่น PWM 20%,40% บนแนวดิ่งที่ไม่ขยับเลย)
        ถ้า target ต่ำกว่าจุดต่ำสุดที่ขยับได้จริง ใช้ PWM ต่ำสุดที่ยังขยับได้แทนการ extrapolate ไปหา 0"""
        if target_speed <= 0:
            return 0.0

        xs, ys = self.get_active_calib_table()

        valid = [(x, y) for x, y in zip(xs, ys) if x > 0]
        if not valid:
            return 1.0
        vx = [p[0] for p in valid]
        vy = [p[1] for p in valid]

        if target_speed <= vx[0]:
            return vy[0]
        if target_speed >= vx[-1]:
            return 1.0

        for i in range(len(vx) - 1):
            if vx[i] <= target_speed <= vx[i + 1]:
                frac = (target_speed - vx[i]) / (vx[i + 1] - vx[i])
                return vy[i] + frac * (vy[i + 1] - vy[i])
        return 1.0

    # =====================================================================
    def compute_pwm(self, target, speed, integ, kp, ki, kickstart_start,
                     last_moving_time, holding_start_time):
        now = time.time()

        if abs(target) < 1e-4:
            if not self.holding_enabled:
                return 0.0, 0.0, kickstart_start, last_moving_time, None, False, False, 0.0

            if holding_start_time is None:
                holding_start_time = now
            hold_elapsed = now - holding_start_time

            if hold_elapsed > self.holding_max_duration:
                hold_pwm = self.holding_pwm * 0.5
            else:
                hold_pwm = self.holding_pwm

            return hold_pwm, 0.0, kickstart_start, last_moving_time, holding_start_time, False, True, 0.0

        holding_start_time = None

        abs_tgt = abs(target)
        abs_spd = abs(speed)

        if abs_spd >= self.kickstart_movement_threshold:
            last_moving_time = now

        recently_moving = (
            last_moving_time is not None
            and (now - last_moving_time) < self.kickstart_grace_period
        )

        if kickstart_start is None and abs_spd < self.kickstart_movement_threshold and not recently_moving:
            kickstart_start = now

        using_kickstart = (
            kickstart_start is not None
            and (now - kickstart_start) < self.kickstart_duration
            and abs_spd < self.kickstart_movement_threshold
        )

        if using_kickstart:
            return self.kickstart_pwm, integ, kickstart_start, last_moving_time, holding_start_time, True, False, 0.0

        if kickstart_start is not None and (abs_spd >= self.kickstart_movement_threshold or recently_moving):
            kickstart_start = None

        err = abs_tgt - abs_spd
        integ = max(-self.integ_limit, min(integ + err * self.speed_dt, self.integ_limit))
        ff = self.speed_to_pwm(abs_tgt)
        pwm = ff + kp * err + ki * integ
        return pwm, integ, kickstart_start, last_moving_time, holding_start_time, False, False, ff

    # =====================================================================
    def control_loop(self):
        if self.manual_pwm_mode:
            self.speed_r, self.pt_r, self.ptime_r = self.calc_speed(self.ticks_r, self.pt_r, self.ptime_r, self.speed_r)
            self.speed_l, self.pt_l, self.ptime_l = self.calc_speed(self.ticks_l, self.pt_l, self.ptime_l, self.speed_l)
            self.rpwm.value = self.manual_pwm_value
            self.lpwm.value = self.manual_pwm_value
            self.rdir.value = False
            self.ldir.value = True
            self.pub_sr.publish(Float32(data=float(self.speed_r)))
            self.pub_sl.publish(Float32(data=float(self.speed_l)))
            self.pub_ticks_r.publish(Float32(data=float(self.ticks_r)))
            self.pub_ticks_l.publish(Float32(data=float(self.ticks_l)))

            self.log_counter += 1
            if self.log_counter >= 20:
                self.log_counter = 0
                self.get_logger().info(
                    f"[MANUAL PWM TEST | surface={self.surface_mode}] pwm={self.manual_pwm_value:.2f} | "
                    f"ticks_R={self.ticks_r} ticks_L={self.ticks_l} | "
                    f"speed_R={self.speed_r*100:.2f}cm/s speed_L={self.speed_l*100:.2f}cm/s"
                )

            if time.time() - self.last_cmd_time > self.cmd_timeout:
                self.manual_pwm_mode = False
                self.rpwm.value = self.lpwm.value = 0.0
            return

        self.speed_r, self.pt_r, self.ptime_r = self.calc_speed(self.ticks_r, self.pt_r, self.ptime_r, self.speed_r)
        self.speed_l, self.pt_l, self.ptime_l = self.calc_speed(self.ticks_l, self.pt_l, self.ptime_l, self.speed_l)

        dpt = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.dist_r = abs(self.ticks_r) * dpt
        self.dist_l = abs(self.ticks_l) * dpt

        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_r = self.target_l = 0.0
            self.integ_r  = self.integ_l  = 0.0
            self.kickstart_start_r = self.kickstart_start_l = None
            self.last_moving_time_r = self.last_moving_time_l = None

        (pwm_r, self.integ_r, self.kickstart_start_r, self.last_moving_time_r,
         self.holding_start_time_r, ks_r, hold_r, ff_r) = self.compute_pwm(
            self.target_r, self.speed_r, self.integ_r, self.kp_r, self.ki_r,
            self.kickstart_start_r, self.last_moving_time_r, self.holding_start_time_r
        )

        (pwm_l, self.integ_l, self.kickstart_start_l, self.last_moving_time_l,
         self.holding_start_time_l, ks_l, hold_l, ff_l) = self.compute_pwm(
            self.target_l, self.speed_l, self.integ_l, self.kp_l, self.ki_l,
            self.kickstart_start_l, self.last_moving_time_l, self.holding_start_time_l
        )

        self._ks_r_active, self._ks_l_active = ks_r, ks_l
        self._hold_r_active, self._hold_l_active = hold_r, hold_l
        self._ff_r, self._ff_l = ff_r, ff_l

        self.rdir.value = self.target_r < 0.0
        self.ldir.value = self.target_l >= 0.0

        self.rpwm.value = max(0.0, min(pwm_r, 1.0))
        self.lpwm.value = max(0.0, min(pwm_l, 1.0))
        self.rdir.value = self.target_r < 0.0
        self.ldir.value = self.target_l >= 0.0

        self.pub_sr.publish(Float32(data=float(self.speed_r)))
        self.pub_sl.publish(Float32(data=float(self.speed_l)))
        self.pub_ticks_r.publish(Float32(data=float(self.ticks_r)))
        self.pub_ticks_l.publish(Float32(data=float(self.ticks_l)))

        self.log_counter += 1
        if self.log_counter >= 20:
            self.log_counter = 0
            note = ""
            if ks_r or ks_l: note += f" | KS(R={ks_r},L={ks_l})"
            if hold_r or hold_l: note += f" | HOLD(R={hold_r},L={hold_l})"
            self.get_logger().info(
                f"[{self.surface_mode}] TGT(cm/s) R={self.target_r*100:.1f} L={self.target_l*100:.1f} | "
                f"ACT(cm/s) R={self.speed_r*100:.1f} L={self.speed_l*100:.1f} | "
                f"FF(%) R={ff_r*100:.0f} L={ff_l*100:.0f}{note}"
            )

    # =====================================================================
    def log_data(self):
        t  = time.time() - self.t0
        er = self.target_r - self.speed_r
        el = self.target_l - self.speed_l
        ang = math.degrees(self.best_angle) if not math.isnan(self.best_angle) else "nan"
        raw_ang = math.degrees(self.raw_angle) if not math.isnan(self.raw_angle) else "nan"
        err_ang = (raw_ang - ang) if (raw_ang != "nan" and ang != "nan") else "nan"
        self.w.writerow([round(t,3), self.cmd_vx, self.cmd_wz,
                         self.target_r, self.target_l,
                         round(self.speed_r,4), round(self.speed_l,4),
                         round(er,4), round(el,4),
                         self.ticks_r, self.ticks_l,
                         round(self.dist_r,4), round(self.dist_l,4),
                         raw_ang,ang,err_ang, self.weld_status,
                         round(getattr(self, '_ff_r', 0.0),4),
                         round(getattr(self, '_ff_l', 0.0),4),
                         getattr(self, '_ks_r_active', False),
                         getattr(self, '_ks_l_active', False),
                         getattr(self, '_hold_r_active', False),
                         getattr(self, '_hold_l_active', False),
                         self.manual_pwm_mode,
                         self.surface_mode])
        self.f.flush()

    # =====================================================================
    def destroy_node(self):
        try:
            self.rpwm.value = self.lpwm.value = 0.0
            for d in [self.rpwm, self.lpwm, self.rdir, self.ldir]: d.close()
            if self.ser and self.ser.is_open: self.ser.close()
            if not self.f.closed: self.f.close()
        finally:
            super().destroy_node()

# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorClosedLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
