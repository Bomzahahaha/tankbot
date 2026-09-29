"""
seam_tracker_pid — ควบคุมทิศทางของหุ่นยนต์ตามมุมรอยเชื่อมที่ตรวจจับได้

Subscribe : /best_angle   (Float32, rad) จาก weld_detector_median
            /weld_status  (String)       WELD_FOUND / NO_WELD
Publish   : /cmd_vel      (Twist)        ไปยัง cmd_vel_to_motor

หลักการ: PD control บนมุม error -> angular.z
         + ลดความเร็วเชิงเส้นแบบ quadratic เมื่อ error มุมสูง (โค้งแรง = ช้าลง)

หมายเหตุการอ่าน comment พารามิเตอร์:
  🔧 = ปรับ/เปลี่ยนค่าจริงหลายครั้งจากการทดสอบ (มีหลักฐานใน git history)
  ⚪ = ตั้งค่าครั้งเดียวตั้งแต่แรก ไม่เคยกลับมาปรับอีกเลยตลอดโปรเจกต์
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class SeamTrackerPID(Node):
    def __init__(self):
        super().__init__('seam_tracker_pid')

        # ⚪ DEAD CODE: ไม่เคยถูก set เป็น True ที่ไหนในระบบตลอดประวัติ git ทั้งหมด
        #    calculate_linear_speed_tank() จึงไม่เคยถูกเรียกใช้จริง — คงไว้เผื่ออนาคต
        self.tank_mode = False

        # ── PID gains (ควบคุมทิศทาง/มุม) ──────────
        self.kp = 4.0     # 🔧 ปรับ 5 ครั้ง (0.15 → 0.25 → 0.8 → 2.5 → 4.0) ยิ่งสูง = เลี้ยวแรง/ไวขึ้น, สูงเกินไป = แกว่ง
        self.ki = 0.0     # 🔧 เคยลอง 0.01 มาก่อน แต่กลับมาปิดที่ 0.0 (ไม่มีผลใดๆ เพราะคูณด้วย 0 เสมอ)
        self.kd = 0.15    # 🔧 ปรับ 4 ครั้ง (0.0 → 0.01 → 0.02 → 0.15) ยิ่งสูง = หน่วงการแกว่งได้ดีขึ้น, สูงเกินไป = ตอบสนองช้า/สั่นจาก noise

        # ── ความเร็ว ──────────────────────────────
        self.max_linear_speed = 0.20   # 🔧 ตัวที่ถูกปรับบ่อยที่สุดในไฟล์นี้ (11 ครั้ง: 0.015→...→0.20) เพดานความเร็วตรงเมื่อ error มุม ≈ 0
        self.min_linear_speed = 0.015  # 🔧 ปรับ 3 ครั้ง (0.005→0.010→0.015) ความเร็วต่ำสุดที่ยังสั่งให้วิ่ง
        self.max_turn_speed   = 0.30   # 🔧 ปรับ 5 ครั้ง (0.04→...→0.30) เพดานความเร็วเลี้ยว
        self.stop_angle_rad   = math.radians(10.0)  # 🔧 ปรับ 1 ครั้ง (12°→10°) error มุมเกินนี้ = ความเร็วตรงตกเป็น 0

        # ── Deadband: error มุมเล็กกว่านี้ถือว่า "ตรงแล้ว" ไม่ต้องเลี้ยว ──
        self.deadband_rad = math.radians(0.2)   # 🔧 ปรับ 4 ครั้ง (1.0°→2.0°→0.5°→0.2°) ยิ่งเล็ก = ไวต่อ error เล็กๆ มากขึ้น, เล็กเกินไป = สั่นจาก noise

        # ── Low-pass filter บน error ก่อนเข้า PID ──
        self.filter_alpha   = 0.2   # 🔧 ปรับ 4 ครั้ง (0.35→0.6→0.4→0.2) ยิ่งสูง = เชื่อค่าเก่ามาก (เรียบขึ้นแต่ช้าลง)
        self.filtered_error = None

        # ── Timeout: ไม่มีมุมใหม่เข้ามานานเกินนี้ = หยุด ──
        self.angle_timeout = 0.3   # ⚪ ไม่เคยปรับเลยตลอดโปรเจกต์ (วินาที)

        # ── NO_WELD: หยุดถ้าไม่เจอเส้นติดต่อกันครบจำนวนนี้ ──
        self.no_weld_count     = 0
        self.no_weld_threshold = 3   # ⚪ ไม่เคยปรับเลย (เฟรม)
        self.no_weld_stopped   = False

        # ── PID state ────────────────────────────
        self.previous_error = 0.0
        self.integral       = 0.0

        self.last_time       = self.get_clock().now()
        self.last_angle_time = self.get_clock().now()

        # ── Slew-rate limiter: จำกัดอัตราการเปลี่ยน angular speed ต่อรอบ ──
        self.prev_angular_speed = 0.0
        self.max_angular_step   = 0.05   # ⚪ ไม่เคยปรับเลย (rad/s ต่อรอบ callback)

        # ── Subscribers ──────────────────────────
        self.angle_subscriber = self.create_subscription(
            Float32, '/best_angle', self.pid_callback, 10
        )
        self.status_subscriber = self.create_subscription(
            String, '/weld_status', self.status_callback, 10
        )

        # ── Publisher ────────────────────────────
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.check_timeout)

        self.get_logger().info('Seam Tracker PID node started')

    def stop_robot(self):
        """สั่งหยุดทันที (linear และ angular = 0)"""
        twist_msg = Twist()
        twist_msg.linear.x  = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.prev_angular_speed = 0.0

    def check_timeout(self):
        """เรียกทุก 0.05s — หยุดถ้าไม่มีมุมใหม่เข้ามานานเกิน angle_timeout"""
        if self.no_weld_stopped:
            self.stop_robot()
            return
        now = self.get_clock().now()
        dt  = (now - self.last_angle_time).nanoseconds / 1e9
        if dt > self.angle_timeout:
            self.stop_robot()

    def status_callback(self, msg: String):
        if msg.data == 'WELD_FOUND':
            self.no_weld_count   = 0
            self.no_weld_stopped = False   # ปลดล็อกการหยุดได้เมื่อกลับมาเจอเส้น

    def calculate_linear_speed(self, error):
        """ความเร็วตรงแบบ quadratic falloff: ยิ่ง error มุมมาก ยิ่งช้าลงเร็ว (ไม่ใช่เชิงเส้น)"""
        error_abs = abs(error)
        if error_abs >= self.stop_angle_rad:
            return 0.0
        ratio = error_abs / self.stop_angle_rad
        speed = self.max_linear_speed * (1.0 - ratio) ** 2
        return max(speed, self.min_linear_speed)

    def calculate_linear_speed_tank(self, error):
        """⚪ DEAD CODE: ใช้เฉพาะเมื่อ tank_mode=True ซึ่งไม่เคยถูกเปิดใช้งานจริง"""
        if abs(error) >= self.stop_angle_rad:
            return 0.0
        return self.min_linear_speed

    def reset_filter(self):
        self.filtered_error = None

    def pid_callback(self, angle_msg: Float32):
        if self.no_weld_stopped:
            self.stop_robot()
            return

        raw_error = angle_msg.data
        self.last_angle_time = self.get_clock().now()

        # ── NO_WELD: NaN/Inf เข้ามา = ไม่เจอเส้นรอบนี้ ──
        if math.isnan(raw_error) or math.isinf(raw_error):
            self.no_weld_count += 1
            self.get_logger().warn(
                f'No seam detected ({self.no_weld_count}/{self.no_weld_threshold})'
            )
            self.stop_robot()

            if self.no_weld_count >= self.no_weld_threshold:
                self.no_weld_stopped = True
                self.get_logger().warn('Confirmed NO_WELD → STOP')
                self.integral       = 0.0
                self.previous_error = 0.0
                self.reset_filter()
            return

        # ── Low-pass filter บน error ──────────────
        if self.filtered_error is None:
            self.filtered_error = raw_error
        else:
            self.filtered_error = (
                self.filter_alpha * self.filtered_error +
                (1.0 - self.filter_alpha) * raw_error
            )
        error = self.filtered_error

        # ── dt ─────────────────────────────────────
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.0:
            dt = 1e-3   # กัน dt=0 หาร error

        # ── PD (I ปิดอยู่เพราะ ki=0) ───────────────
        p = self.kp * error

        self.integral += error * dt
        self.integral  = max(min(self.integral, 1.0), -1.0)  # anti-windup clamp
        i = self.ki * self.integral

        derivative = (error - self.previous_error) / dt
        d = self.kd * derivative

        angular_speed = -(p + i + d)
        angular_speed = max(min(angular_speed, self.max_turn_speed), -self.max_turn_speed)

        # Slew-rate limiter: จำกัดการเปลี่ยนแปลงต่อรอบไม่ให้กระชาก
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

        # ── ความเร็วตรง ────────────────────────────
        if self.tank_mode:   # ⚪ ไม่เคยเป็น True จริง — เก็บไว้เผื่ออนาคต
            linear_speed = self.calculate_linear_speed_tank(error)
        else:
            linear_speed = self.calculate_linear_speed(error)

        if raw_in_db or filtered_in_db:
            # อยู่ใน deadband แล้ว (มุมตรงพอ) → ใช้ความเร็วเต็มที่แทน (ไม่ผ่านสูตร falloff)
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
