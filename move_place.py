#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf
from pymycobot import MyCobotSocket

# ------------------------------------------------------------------
# ค่าคงที่สำหรับแขน (ปรับตาม calibration จริง)
# ------------------------------------------------------------------
# ตำแหน่งวาง (พิกัดฐานแขน) [x, y, z, rx, ry, rz] หน่วย mm และ องศา
PLACE_COORDS = [89.2, -64.2, 194.6, 179.98, 0.99, -46.58]

# ความเร็วแขน (0-100)
ARM_SPEED = 20

# IP และ Port ของแขน (ฝั่งแขนต้องเปิด server ไว้)
ARM_IP = "192.168.137.119"
ARM_PORT = 9000

# ------------------------------------------------------------------
# คลาสควบคุม AGV ด้วย Odometry
# ------------------------------------------------------------------
class AGVOdomController:
    def __init__(self):
        rospy.loginfo("Initializing AGV odometry controller...")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        """อ่านตำแหน่งและมุมจาก odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        euler = tf.euler_from_quaternion(quat)
        self.current_yaw = euler[2]
        self.odom_received = True

    def move_forward(self, distance_meters, speed=0.15):
        """เคลื่อนที่ไปข้างหน้าตามระยะทางที่กำหนด (เมตร) โดยใช้ odometry"""
        rospy.loginfo(f"Moving forward {distance_meters:.2f} m")

        # รอ odometry
        timeout = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and not self.odom_received and rospy.Time.now() < timeout:
            self.rate.sleep()
        if not self.odom_received:
            rospy.logerr("No odometry data. Aborting move.")
            return False

        start_x = self.current_x
        start_y = self.current_y
        traveled = 0.0
        twist = Twist()
        twist.linear.x = speed

        while not rospy.is_shutdown() and traveled < distance_meters:
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)

            rospy.loginfo("Traveled: %.3f / %.3f m", traveled, distance_meters)
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # หยุด
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Reached target distance. Final traveled: %.3f m", traveled)
        return True


# ------------------------------------------------------------------
# คลาสควบคุมแขนผ่าน WiFi (เฉพาะวาง ไม่ใช้กล้อง)
# ------------------------------------------------------------------
class ArmController:
    def __init__(self):
        rospy.loginfo(f"Connecting to arm via WiFi at {ARM_IP}:{ARM_PORT}...")
        try:
            self.mc = MyCobotSocket(ARM_IP, ARM_PORT)
            rospy.loginfo("Arm connected")
        except Exception as e:
            rospy.logerr(f"Arm connection failed: {e}")
            self.mc = None
            return

    def move_to_home(self):
        """ส่งแขนไปท่า home (angles ทั้งหมด 0)"""
        if not self.mc:
            return
        rospy.loginfo("Moving arm to home pose...")
        self.mc.set_gripper_state(0, 100) 
        rospy.sleep(5)
        self.mc.send_angles([0, 45, -120, -13, 0, -45],ARM_SPEED)
        rospy.sleep(5)
        
        print("สั่ง gripper หนีบ...")
        self.mc.set_gripper_state(1, 100)   # 1 = ปิด
        rospy.sleep(5)

    def place_object(self):
        """
        สั่งให้แขนเคลื่อนที่ไปยังตำแหน่งวางที่กำหนดและปล่อย gripper
        """
        if not self.mc:
            rospy.logerr("Arm not connected")
            return False

        rospy.loginfo(f"Moving arm to place position: {PLACE_COORDS}")
        self.mc.send_coords([257.6, -60.4, 300.0, 177.34, 0.0, -45.28], ARM_SPEED)
        rospy.sleep(5)  # ปรับตามระยะทางจริง
        self.mc.send_coords([257.6, -60.4, 100.0, 177.34, 0.0, -45.28], ARM_SPEED)
        rospy.loginfo("Opening gripper to release object...")
        self.mc.set_gripper_state(0, 100)   # 0 = เปิด
        rospy.sleep(2)

        # (Optional) ยกแขนขึ้นหลังจากวาง
        lift_coords = PLACE_COORDS.copy()
        lift_coords[2] = 300.0  # ยก Z เป็น 300 mm
        rospy.loginfo(f"Lifting arm to height {lift_coords[2]} mm...")
        self.mc.send_coords([257.6, -60.4, 500.0, 177.34, 0.0, -45.28], ARM_SPEED)
        rospy.sleep(4)
        self.mc.send_angles([0, 45, -120, -13, 0, -45],ARM_SPEED)
        
        rospy.loginfo("Place operation completed")
        return True


# ------------------------------------------------------------------
# คลาสหลัก รวม AGV + Arm (ไม่ใช้กล้อง)
# ------------------------------------------------------------------
class AGVWithArmPlace:
    def __init__(self):
        rospy.init_node('agv_with_arm_place')

        self.agv = AGVOdomController()
        self.arm = ArmController()

        # ระยะที่ AGV ต้องเคลื่อนที่ (เมตร) สามารถปรับได้ผ่าน parameter
        self.move_distance = rospy.get_param('~move_distance', 1.0)

    def run(self):
        # 1. แขนไป home (เตรียมพร้อม)
        self.arm.move_to_home()
        rospy.sleep(1)

        # 2. AGV เดินหน้า
        rospy.loginfo(f"AGV moving forward {self.move_distance} meter(s)...")
        if not self.agv.move_forward(self.move_distance):
            rospy.logerr("AGV movement failed. Exiting.")
            return

        # 3. เมื่อถึงระยะแล้ว ให้แขนทำงานวางของ
        rospy.loginfo("AGV reached target distance. Arm performing place operation...")
        self.arm.place_object()

        rospy.loginfo("Mission completed.")


if __name__ == '__main__':
    try:
        node = AGVWithArmPlace()
        node.run()
    except rospy.ROSInterruptException:
        pass