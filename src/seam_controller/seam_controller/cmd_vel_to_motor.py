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

        # --- Feedforward แยกฝั่ง (ชดเชยความต่างของมอเตอร์) ---
        self.max_spd_r = 0.22   # ล้อขวk kff
        self.max_spd_l = 0.22   # ล้อซ้าย kff
        self.pwm_offset_r = 0.05  # จาก min_pwm test
        self.pwm_offset_l = 0.07  # จาก min_pwm test
        
        # --- PID แยกฝั่ง (Independent) ---
        self.kp_r = 3.0;  self.ki_r = 0.35
        self.kp_l = 3.5;  self.ki_l = 0.40

        self.integ_limit = 0.5
        self.speed_dt    = 0.1
        self.alpha       = 0.6
        self.cmd_timeout = 0.5

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
        self.weld_status = "UNKNOWN"

        # --- Logger ---
        os.makedirs("logs", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.f = open(f"logs/log_{ts}.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t","vx","wz","tgt_r","tgt_l",
                         "spd_r","spd_l","err_r","err_l",
                         "ticks_r","ticks_l","dist_r","dist_l",
                         "angle","weld"])

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
        self.create_subscription(String, "/weld_status", self.on_weld,  10)
        self.pub_sr = self.create_publisher(Float32, "/right_wheel_speed", 10)
        self.pub_sl = self.create_publisher(Float32, "/left_wheel_speed",  10)

        self.create_timer(0.01, self.read_serial)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.05, self.log_data)
        self.get_logger().info("🚀 TankBot Dual-PID Node Started")

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
    def on_weld (self, msg): self.weld_status = msg.data

    # =====================================================================
    def calc_speed(self, ticks, pt, ptime, spd):
        now = time.time()
        if pt is None: return 0.0, ticks, now
        dt = now - ptime
        if dt < self.speed_dt: return spd, pt, ptime
        delta = ticks - pt

        # กรอง encoder ที่กระโดดผิดปกติ
        max_ticks = (0.22 / (2.0 * math.pi * self.wheel_radius)) \
                    * self.ticks_per_rev * dt * 2.0
        if abs(delta) > max_ticks:
            return spd * 0.8, ticks, now  # ทิ้งค่านี้ ใช้ค่าเดิมค่อยๆ ลด

        if delta == 0:
            spd = spd * 0.8
            if abs(spd) < 0.001: spd = 0.0
            return spd, ticks, now

        dist = (delta / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius
        raw  = dist / dt
        spd  = self.alpha * spd + (1.0 - self.alpha) * raw
        return spd, ticks, now

    # =====================================================================
    def control_loop(self):
        # 1. อัปเดตความเร็วจริง
        self.speed_r, self.pt_r, self.ptime_r = self.calc_speed(self.ticks_r, self.pt_r, self.ptime_r, self.speed_r)
        self.speed_l, self.pt_l, self.ptime_l = self.calc_speed(self.ticks_l, self.pt_l, self.ptime_l, self.speed_l)

        # 2. ระยะทางสะสม
        dpt = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.dist_r = abs(self.ticks_r) * dpt
        self.dist_l = abs(self.ticks_l) * dpt

        # 3. Timeout
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_r = self.target_l = 0.0
            self.integ_r  = self.integ_l  = 0.0

        # PID ล้อขวา
        if abs(self.target_r) < 1e-4:
            pwm_r = 0.0; self.integ_r = 0.0
        else:
            # ✅ ใช้ abs ทั้งคู่ — direction จัดการโดย rdir แยกต่างหาก
            abs_tgt_r = abs(self.target_r)
            abs_spd_r = abs(self.speed_r)
            err_r = abs_tgt_r - abs_spd_r
            self.integ_r = max(-self.integ_limit,
                           min(self.integ_r + err_r * self.speed_dt, self.integ_limit))
            ff_r  = self.pwm_offset_r + (1.0 - self.pwm_offset_r) * (abs_tgt_r / self.max_spd_r)
            pwm_r = ff_r + self.kp_r * err_r + self.ki_r * self.integ_r

        # PID ล้อซ้าย
        if abs(self.target_l) < 1e-4:
            pwm_l = 0.0; self.integ_l = 0.0
        else:
            abs_tgt_l = abs(self.target_l)
            abs_spd_l = abs(self.speed_l)
            err_l = abs_tgt_l - abs_spd_l
            self.integ_l = max(-self.integ_limit,
                           min(self.integ_l + err_l * self.speed_dt, self.integ_limit))
            ff_l  = self.pwm_offset_l + (1.0 - self.pwm_offset_l) * (abs_tgt_l / self.max_spd_l)
            pwm_l = ff_l + self.kp_l * err_l + self.ki_l * self.integ_l

        # direction แยกออกมาต่างหาก
        self.rdir.value = self.target_r < 0.0
        self.ldir.value = self.target_l >= 0.0

        # 6. ส่ง PWM ออก Hardware
        self.rpwm.value = max(0.0, min(pwm_r, 1.0))
        self.lpwm.value = max(0.0, min(pwm_l, 1.0))
        self.rdir.value = self.target_r < 0.0
        self.ldir.value = self.target_l >= 0.0

        # 7. Publish
        self.pub_sr.publish(Float32(data=float(self.speed_r)))
        self.pub_sl.publish(Float32(data=float(self.speed_l)))

        # 8. INFO
        self.log_counter += 1
        if self.log_counter >= 20:
            self.log_counter = 0
            self.get_logger().info(
                f"TGT(cm/s) R={self.target_r*100:.1f} L={self.target_l*100:.1f} | "
                f"ACT(cm/s) R={self.speed_r*100:.1f} L={self.speed_l*100:.1f} | "
                f"DIST(cm)  R={self.dist_r*100:.1f} L={self.dist_l*100:.1f}"
            )

    # =====================================================================
    def log_data(self):
        t  = time.time() - self.t0
        er = self.target_r - self.speed_r
        el = self.target_l - self.speed_l
        ang = math.degrees(self.best_angle) if not math.isnan(self.best_angle) else "nan"
        self.w.writerow([round(t,3), self.cmd_vx, self.cmd_wz,
                         self.target_r, self.target_l,
                         round(self.speed_r,4), round(self.speed_l,4),
                         round(er,4), round(el,4),
                         self.ticks_r, self.ticks_l,
                         round(self.dist_r,4), round(self.dist_l,4),
                         ang, self.weld_status])
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
