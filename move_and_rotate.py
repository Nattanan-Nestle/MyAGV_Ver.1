#!/usr/bin/env python3
"""
move_and_rotate.py

สั่งให้หุ่นยนต์:
1. เคลื่อนที่ไปข้างหน้า 1.0 - 1.2 เมตร (กำหนดในตัวแปร distance)
2. หมุน 360 องศา
3. เคลื่อนที่ไปข้างหน้าอีก 1.0 - 1.2 เมตร

ใช้ Odometry ควบคุมระยะทางและมุม
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf

class MoveAndRotate:
    def __init__(self):
        rospy.init_node('move_and_rotate')

        # พารามิเตอร์
        self.distance = 1.0          # เมตร (สามารถปรับเป็น 1.0-1.2 ตามต้องการ)
        self.rotate_angle = 360.0    # องศา
        self.speed = 0.15             # ความเร็วเดินหน้า (m/s)
        self.angular_speed = 0.5      # ความเร็วหมุน (rad/s) ≈ 28.6 องศา/วินาที
        self.tolerance = 0.02          # tolerance ระยะทาง (เมตร)
        self.angle_tolerance = 0.05    # tolerance มุม (เรเดียน) ≈ 2.86 องศา
        self.control_rate = 20         # Hz

        # ตัวแปรภายใน
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Publisher / Subscriber
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(self.control_rate)

        rospy.loginfo("MoveAndRotate node started. Waiting for odometry...")

    def odom_callback(self, msg):
        """อ่านตำแหน่งและมุมจาก odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # แปลง quaternion → euler
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        euler = tf.euler_from_quaternion(quat)
        self.current_yaw = euler[2]   # yaw ในช่วง [-π, π]

        self.odom_received = True

    def move_forward(self, distance):
        """เคลื่อนที่ไปข้างหน้าตามระยะทาง (เมตร)"""
        rospy.loginfo(f"Start moving forward {distance:.2f} m")

        # รอให้มี odometry ก่อน
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and not self.odom_received and rospy.Time.now() < timeout:
            self.rate.sleep()
        if not self.odom_received:
            rospy.logerr("No odometry data. Aborting move_forward.")
            return False

        start_x = self.current_x
        start_y = self.current_y
        traveled = 0.0

        twist = Twist()
        twist.linear.x = self.speed

        while not rospy.is_shutdown() and traveled < distance - self.tolerance:
            # คำนวณระยะทางที่เคลื่อนที่ไปแล้ว (Euclidean distance)
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)

            rospy.loginfo("Traveled: %.3f m / %.3f m", traveled, distance)

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # หยุด
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Reached target distance. Final traveled: %.3f m", traveled)
        return True

    def rotate(self, angle_degrees):
        """หมุนตามมุมที่กำหนด (องศา) ค่า + = ทวนเข็ม, - = ตามเข็ม"""
        rospy.loginfo(f"Start rotating {angle_degrees:.1f} degrees")

        # รอ odometry
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and not self.odom_received and rospy.Time.now() < timeout:
            self.rate.sleep()
        if not self.odom_received:
            rospy.logerr("No odometry data. Aborting rotate.")
            return False

        target_angle_rad = math.radians(angle_degrees)
        last_yaw = self.current_yaw
        angle_accumulated = 0.0

        twist = Twist()
        twist.angular.z = self.angular_speed if target_angle_rad >= 0 else -self.angular_speed

        while not rospy.is_shutdown() and abs(angle_accumulated) < abs(target_angle_rad) - self.angle_tolerance:
            # คำนวณ delta yaw แบบ unwrap
            delta = self.current_yaw - last_yaw
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            angle_accumulated += delta
            last_yaw = self.current_yaw

            rospy.loginfo("Rotated: %.2f° / %.2f°", math.degrees(angle_accumulated), angle_degrees)

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # หยุดหมุน
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Rotation completed. Final angle: %.2f°", math.degrees(angle_accumulated))
        return True

    def execute_sequence(self):
        """ดำเนินการตามลำดับ"""
        rospy.sleep(1)  # รอให้ odometry พร้อม

        # 1. เคลื่อนที่ไปข้างหน้าครั้งแรก
        if not self.move_forward(self.distance):
            return

        rospy.sleep(1)  # หยุดพักระหว่างภารกิจ

        # 2. หมุน 360 องศา
        if not self.rotate(self.rotate_angle):
            return

        rospy.sleep(1)

        # 3. เคลื่อนที่ไปข้างหน้าครั้งที่สอง
        if not self.move_forward(self.distance):
            return

        rospy.loginfo("All tasks completed successfully.")

if __name__ == '__main__':
    try:
        node = MoveAndRotate()
        node.execute_sequence()
    except rospy.ROSInterruptException:
        pass