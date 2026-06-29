import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class DriveDistance(Node):
    def __init__(self):
        super().__init__('drive_distance')

        # ============================
        # ตั้งค่าตรงนี้
        # ============================
        self.target_distance = 1.00      # เมตร
        self.drive_speed     = 0.015    # m/s

        # ค่าจาก cmd_vel_to_motor.py
        self.ticks_per_rev   = 259      # OMRON E6A2-C
        self.wheel_radius    = 0.0365   # เมตร

        # คำนวณ Ticks ที่ต้องการ
        wheel_circumference     = 2 * 3.14159 * self.wheel_radius
        revs_needed             = self.target_distance / wheel_circumference
        self.target_ticks       = int(revs_needed * self.ticks_per_rev)

        self.get_logger().info(
            f'เป้าหมาย {self.target_distance}m = {self.target_ticks} ticks'
        )

        # ============================
        # ตัวแปร State
        # ============================
        self.start_ticks  = None
        self.current_ticks = 0
        self.done         = False

        # ============================
        # ROS2
        # ============================
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub = self.create_subscription(
            Int32,
            '/right_encoder_ticks',
            self.encoder_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('รอ Encoder...')

    def encoder_callback(self, msg: Int32):
        self.current_ticks = msg.data

        # บันทึก Ticks ตั้งต้น
        if self.start_ticks is None:
            self.start_ticks = self.current_ticks
            self.get_logger().info(
                f'เริ่มที่ Ticks = {self.start_ticks}'
            )

    def control_loop(self):
        if self.done:
            return

        if self.start_ticks is None:
            return

        # คำนวณ Ticks ที่วิ่งไปแล้ว
        ticks_traveled = abs(self.current_ticks - self.start_ticks)
        distance_traveled = (
            ticks_traveled / self.ticks_per_rev
        ) * (2 * 3.14159 * self.wheel_radius)

        self.get_logger().info(
            f'Ticks: {ticks_traveled}/{self.target_ticks} | '
            f'ระยะ: {distance_traveled:.3f}/{self.target_distance}m'
        )

        msg = Twist()

        if ticks_traveled >= self.target_ticks:
            # ถึงเป้าแล้ว — หยุด!
            msg.linear.x = 0.0
            self.pub.publish(msg)
            self.done = True
            self.get_logger().info(
                f'✅ ถึง {self.target_distance}m แล้ว! หยุด'
            )
        else:
            # ยังไม่ถึง — เดินหน้าต่อ
            msg.linear.x = self.drive_speed
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = DriveDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # หยุดมอเตอร์ทันทีถ้ากด Ctrl+C
        stop = Twist()
        node.pub.publish(stop)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
