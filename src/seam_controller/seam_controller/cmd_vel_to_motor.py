import csv
import math
import os
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32, String
from gpiozero import PWMOutputDevice, DigitalOutputDevice

class CmdVelToMotorClosedLoop(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_motor")
        self.wheel_base = 0.30
        self.wheel_radius = 0.0295
        self.ticks_per_rev = 200
        self.max_wheel_speed = 0.16
        self.min_pwm = 0.0
        self.cmd_timeout = 0.5
        self.kp = 0.6
        self.ki = 0.03
        self.integ_limit = 0.15
        self.alpha = 0.7
        self.speed_dt = 0.2
        self.last_cmd_time = time.time()
        self.target_r = 0.0
        self.target_l = 0.0
        self.ff_r = 0.0
        self.ff_l = 0.0
        self.dir_r = True
        self.dir_l = True
        self.ticks_r = 0
        self.ticks_l = 0
        self.dist_r = 0
        self.dist_l = 0
        self.pt_r = None
        self.ptime_r = None
        self.speed_r = 0.0
        self.integ_r = 0.0
        self.pt_l = None
        self.ptime_l = None
        self.speed_l = 0.0
        self.integ_l = 0.0
        self.t0 = time.time()
        self.log_counter = 0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.best_angle = float("nan")
        self.weld_status = "UNKNOWN"
        self.mae_r = []
        self.mae_l = []
        self.offset_r = None
        self.offset_l = None
        os.makedirs("logs", exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.f = open(f"logs/log_{ts}.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t","vx","wz","target_r","target_l","speed_r","speed_l","err_r","err_l","ticks_r","ticks_l","dist_r ","dist_l","angle","weld"])
        self.lpwm = PWMOutputDevice(18, frequency=1000, initial_value=0.0)
        self.ldir = DigitalOutputDevice(17, initial_value=False)
        self.rpwm = PWMOutputDevice(19, frequency=1000, initial_value=0.0)
        self.rdir = DigitalOutputDevice(26, initial_value=False)
        self.ser = None
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
            time.sleep(2.0)
            self.get_logger().info("Arduino connected")
        except Exception as e:
            self.get_logger().warn(f"Serial failed: {e}")
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float32, "/best_angle", self.on_angle, 10)
        self.create_subscription(String, "/weld_status", self.on_weld, 10)
        self.pub_tr = self.create_publisher(Int32, "/right_encoder_ticks", 10)
        self.pub_tl = self.create_publisher(Int32, "/left_encoder_ticks", 10)
        self.pub_sr = self.create_publisher(Float32, "/right_wheel_speed", 10)
        self.pub_sl = self.create_publisher(Float32, "/left_wheel_speed", 10)
        self.create_timer(0.01, self.read_serial)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.05, self.log_data)
        self.create_timer(1.0, self.summary)
        self.get_logger().info("cmd_vel_to_motor ready")

    def read_serial(self):
        if not self.ser:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if "," not in line:
                    continue
                for p in line.split(","):
                    p = p.strip()
                    if p.startswith("R:"):
                        raw = int(p[2:])
                        if self.offset_r is None:
                            self.offset_r = raw
                        self.ticks_r = raw - self.offset_r
                    elif p.startswith("L:"):
                        raw = int(p[2:])
                        if self.offset_l is None:
                            self.offset_l = raw
                        self.ticks_l = raw - self.offset_l
        except Exception as e:
            self.get_logger().warn(f"Serial: {e}")

    def on_cmd(self, msg):
        self.last_cmd_time = time.time()
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z
        v, w = msg.linear.x, msg.angular.z
        self.target_r = v + w * self.wheel_base / 2.0
        self.target_l = v - w * self.wheel_base / 2.0
        self.ff_r, self.dir_r = self.vel2pwm(self.target_r)
        self.ff_l, self.dir_l = self.vel2pwm(self.target_l)

    def on_angle(self, msg):
        self.best_angle = msg.data

    def on_weld(self, msg):
        self.weld_status = msg.data

    def vel2pwm(self, v):
        fwd = v >= 0.0
        pwm = abs(v) / self.max_wheel_speed
        pwm = max(0.0, min(pwm, 1.0))
        if pwm > 0.0:
            pwm = self.min_pwm + (1.0 - self.min_pwm) * pwm
        return min(pwm, 1.0), fwd

    def calc_speed(self, ticks, pt, ptime, spd):
        now = time.time()
        if pt is None:
            return 0.0, ticks, now
        dt = now - ptime
        if dt < self.speed_dt:
            return spd, pt, ptime
        delta = ticks - pt
        if abs(delta) <= 0:
            spd = spd * 0.8
            if abs(spd) < 0.001:
                spd = 0.0
            return spd, ticks, now
        dist = (ticks - pt) / self.ticks_per_rev * 2.0 * math.pi * self.wheel_radius
        raw = dist / dt
        spd = self.alpha * spd + (1.0 - self.alpha) * raw
        return spd, ticks, now

    def control_loop(self):
        self.speed_r, self.pt_r, self.ptime_r = self.calc_speed(self.ticks_r, self.pt_r, self.ptime_r, self.speed_r)
        self.speed_l, self.pt_l, self.ptime_l = self.calc_speed(self.ticks_l, self.pt_l, self.ptime_l, self.speed_l)
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_r = self.target_l = 0.0
            self.ff_r = self.ff_l = 0.0
            self.integ_r = self.integ_l = 0.0
        if abs(self.target_r) < 1e-4:
            pwm_r = 0.0
            self.integ_r = 0.0
        else:
            err = self.target_r - self.speed_r
            self.integ_r = max(-self.integ_limit, min(self.integ_r + err * self.speed_dt, self.integ_limit))
            pwm_r = self.ff_r + self.kp * err + self.ki * self.integ_r
        if abs(self.target_l) < 1e-4:
            pwm_l = 0.0
            self.integ_l = 0.0
        else:
            err = self.target_l - self.speed_l
            self.integ_l = max(-self.integ_limit, min(self.integ_l + err * self.speed_dt, self.integ_limit))
            pwm_l = self.ff_l + self.kp * err + self.ki * self.integ_l
        pwm_r = max(0.0, min(pwm_r, 1.0))

        pwm_l = max(0.0, min(pwm_l, 1.0))
        self.rdir.value = not self.dir_r
        self.ldir.value = self.dir_l
        self.rpwm.value = pwm_r
        self.lpwm.value = pwm_l
        self.pub_tr.publish(Int32(data=self.ticks_r))
        self.pub_tl.publish(Int32(data=self.ticks_l))
        self.pub_sr.publish(Float32(data=float(self.speed_r)))
        self.pub_sl.publish(Float32(data=float(self.speed_l)))
        dist_per_tick   = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.dist_r     = abs(self.ticks_r) * dist_per_tick
        self.dist_l     = abs(self.ticks_l) * dist_per_tick
        self.log_counter += 1
        if self.log_counter >= 20:
            self.log_counter = 0
            self.get_logger().info(
                f"spd R={self.speed_r:.3f} L={self.speed_l:.3f} m/s | "
                f"dist R={self.dist_r:.3f} L={self.dist_l:.3f} m/s |"
                f"pwm_r={pwm_r:.2f} pwm_l={pwm_l:.2f} "
            )

    def log_data(self):
        t = time.time() - self.t0
        er = self.target_r - self.speed_r
        el = self.target_l - self.speed_l
        ang = math.degrees(self.best_angle) if not math.isnan(self.best_angle) else "nan"
        self.mae_r.append(abs(er))
        self.mae_l.append(abs(el))
        self.w.writerow([round(t,3), self.cmd_vx, self.cmd_wz, self.target_r, self.target_l, round(self.speed_r,4), round(self.speed_l,4), round(er,4), round(el,4), self.ticks_r, self.ticks_l, round(self.dist_r,4), round(self.dist_l,4), ang, self.weld_status])
        self.f.flush()

    def summary(self):
        if not self.mae_r:
            return
        self.get_logger().info(f"MAE R={sum(self.mae_r)/len(self.mae_r):.4f} L={sum(self.mae_l)/len(self.mae_l):.4f}")

    def destroy_node(self):
        try:
            self.rpwm.value = 0.0
            self.lpwm.value = 0.0
            for d in [self.rpwm, self.lpwm, self.rdir, self.ldir]:
                d.close()
            if self.ser and self.ser.is_open:
                self.ser.close()
            if not self.f.closed:
                self.f.close()
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMotorClosedLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
