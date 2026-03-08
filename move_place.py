#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf
from pymycobot import MyCobotSocket   # ใช้สำหรับเชื่อมต่อ WiFi


class AGVMoveAndArmPlace:
    def __init__(self):
        rospy.init_node('agv_move_and_arm_place')

        # ---------- พารามิเตอร์ที่ปรับได้ ----------
        # ระยะ AGV เคลื่อนที่ (เมตร)
        self.agv_distance = rospy.get_param('~agv_distance', 1.0)
        # ความเร็ว AGV (m/s)
        self.agv_speed = rospy.get_param('~agv_speed', 0.15)

        # IP และ Port ของแขน (ฝั่งแขนต้องเปิด server ไว้)
        self.arm_ip = rospy.get_param('~arm_ip', '192.168.137.119')
        self.arm_port = rospy.get_param('~arm_port', 9000)
        # ความเร็วแขน (0-100)
        self.arm_speed = rospy.get_param('~arm_speed', 20)

        # ตำแหน่งวาง (พิกัดฐานแขน) [x, y, z, rx, ry, rz] หน่วย mm และ องศา
        self.place_coords = rospy.get_param(
            '~place_coords',
            [89.2, -64.2, 194.6, 179.98, 0.99, -46.58]
        )
        # ความสูงที่ยกขึ้นหลังจากวาง (mm)
        self.lift_height = rospy.get_param('~lift_height', 300.0)

        # ---------- ส่วนควบคุม AGV ด้วย Odometry ----------
        rospy.loginfo("Initializing AGV odometry controller...")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.rate = rospy.Rate(10)

        # ---------- ส่วนควบคุมแขนกลผ่าน WiFi ----------
        rospy.loginfo(f"Connecting to arm via WiFi at {self.arm_ip}:{self.arm_port}...")
        try:
            self.mc = MyCobotSocket(self.arm_ip, self.arm_port)
            # รอให้การเชื่อมต่อมั่นคง
            rospy.sleep(2)
            rospy.loginfo("Arm connected successfully")
        except Exception as e:
            rospy.logerr(f"Arm connection failed: {e}")
            rospy.signal_shutdown("Arm connection failed")
            return

    def odom_callback(self, msg):
        """อ่านตำแหน่งและมุมจาก odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        euler = tf.euler_from_quaternion(quat)
        self.current_yaw = euler[2]
        self.odom_received = True

    def move_forward(self, distance, speed):
        """
        เคลื่อนที่ไปข้างหน้าตามระยะทางที่กำหนด (เมตร)
        ใช้ odometry วัดระยะทางจริง
        """
        rospy.loginfo(f"AGV moving forward {distance:.2f} m at speed {speed:.2f} m/s")

        # รอ odometry
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and not self.odom_received and rospy.Time.now() < timeout:
            rospy.loginfo_throttle(1, "Waiting for odometry...")
            self.rate.sleep()
        if not self.odom_received:
            rospy.logerr("No odometry data received. Aborting move.")
            return False

        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw
        traveled = 0.0
        twist = Twist()
        twist.linear.x = speed

        while not rospy.is_shutdown() and traveled < distance:
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            cos_yaw = math.cos(start_yaw)
            sin_yaw = math.sin(start_yaw)
            forward_body = cos_yaw * dx + sin_yaw * dy
            traveled = forward_body
            rospy.loginfo("Traveled: %.3f / %.3f m", traveled, distance)
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Reached target distance. Final traveled: %.3f m", traveled)
        return True

    def arm_place(self):
        """สั่งแขนผ่าน WiFi ไปวางที่ตำแหน่งและปล่อย gripper"""
        if not self.mc:
            rospy.logerr("Arm not connected")
            return False

        rospy.loginfo("Moving arm to place position...")
        rospy.loginfo(f"Target coords: {self.place_coords}")

        # สั่งเคลื่อนที่ (send_coords ไม่รอให้เสร็จ ต้องมี sleep ประมาณ)
        self.mc.send_coords(self.place_coords, self.arm_speed)
        rospy.sleep(5)   # ปรับตามระยะทางจริง

        rospy.loginfo("Opening gripper to release object...")
        self.mc.set_gripper_state(0, 100)   # 0 = เปิด
        rospy.sleep(2)

        # ยกแขนขึ้นหลังจากปล่อย
        lift_coords = self.place_coords.copy()
        lift_coords[2] = self.lift_height
        rospy.loginfo(f"Lifting arm to height {self.lift_height} mm...")
        self.mc.send_coords(lift_coords, self.arm_speed)
        rospy.sleep(4)

        rospy.loginfo("Arm place operation completed")
        return True

    def run(self):
        """ลำดับการทำงานหลัก"""
        rospy.loginfo("=== Starting AGV movement ===")
        if not self.move_forward(self.agv_distance, self.agv_speed):
            rospy.logerr("AGV movement failed. Exiting.")
            return

        rospy.loginfo("=== AGV reached target. Arm performing place operation ===")
        self.arm_place()

        rospy.loginfo("=== Mission completed successfully ===")


if __name__ == '__main__':
    try:
        node = AGVMoveAndArmPlace()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")