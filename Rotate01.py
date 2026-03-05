#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf

class RotateByAngle:
    def __init__(self):
        rospy.init_node('rotate_by_angle')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_yaw = 0.0          # มุมปัจจุบันจาก odom (เรเดียน)
        self.last_yaw = None             # ค่ามุมครั้งก่อน (สำหรับคำนวณ delta)
        self.angle_accumulated = 0.0    # มุมที่หมุนไปแล้วสะสม (เรเดียน)
        self.target_angle = 0.0          # มุมเป้าหมาย (เรเดียน)
        self.rotating = False

    def odom_callback(self, msg):
        # แปลง quaternion → euler เพื่อเอา yaw
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        euler = tf.euler_from_quaternion(quat)
        self.current_yaw = euler[2]      # yaw ในช่วง [-π, π]

    def rotate(self, angle_degrees, angular_speed=0.5):
        """
        หมุนหุ่นยนต์ตามมุมที่กำหนด (องศา) เทียบกับทิศทางปัจจุบัน
        angle_degrees > 0 : หมุนทวนเข็มนาฬิกา (CCW)
        angle_degrees < 0 : หมุนตามเข็มนาฬิกา (CW)
        angular_speed : ความเร็วเชิงมุม (rad/s) ค่าเริ่มต้น 0.5 rad/s ≈ 28.6 องศา/วินาที
        """
        self.target_angle = math.radians(angle_degrees)
        self.last_yaw = self.current_yaw
        self.angle_accumulated = 0.0
        self.rotating = True

        twist = Twist()
        twist.angular.z = angular_speed if self.target_angle >= 0 else -angular_speed

        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown() and self.rotating:
            # คำนวณ delta yaw แบบ unwrap (จัดการกรณีข้าม ±π)
            delta = self.current_yaw - self.last_yaw
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            self.angle_accumulated += delta
            self.last_yaw = self.current_yaw

            rospy.loginfo("หมุนไปแล้ว: %.2f° / เป้าหมาย: %.2f°",
                          math.degrees(self.angle_accumulated), angle_degrees)

            # หยุดเมื่อถึงเป้าหมาย (เผื่อ tolerance 0.01 rad ≈ 0.57°)
            if abs(self.angle_accumulated) >= abs(self.target_angle) - 0.01:
                self.rotating = False
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("ถึงเป้าหมายแล้ว!")
                break

            self.cmd_pub.publish(twist)
            rate.sleep()

        # หยุดอีกครั้งเพื่อความปลอดภัย
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        rotator = RotateByAngle()
        rospy.sleep(1)  # รอให้ odometry พร้อม

        # ตัวอย่าง: หมุน 360 องศา (กลับมาทางเดิม)
        rotator.rotate(360.0)

        # สามารถเปลี่ยนค่าตามต้องการ เช่น 315 องศา หรือ 405 องศา
        # rotator.rotate(315.0)
        # rotator.rotate(405.0)

    except rospy.ROSInterruptException:
        pass