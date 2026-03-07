#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations as tf
from pymycobot import MyCobotSocket

# ------------------------------------------------------------------
# ค่าคงที่สำหรับแขน (ปรับตาม calibration จริง)
# ------------------------------------------------------------------
SCALE = 0.4092           # mm/pixel
CX = 320
CY = 240
OFFSET_X = 265           # mm offset
OFFSET_Y = -59
FIXED_RX = -178.69
FIXED_RY = -0.29
FIXED_RZ = -45.26

COLOR_RANGES = {
    "Green":    [(71, 211, 188),  (82, 255, 228)],
    "Red":      [(164, 154, 221), (179, 255, 255)],
}

SPEED = 20                # ความเร็วแขน
ARM_IP = "192.168.137.119"   # IP ของแขน (เปลี่ยนให้ตรง)
ARM_PORT = 9000

# ------------------------------------------------------------------
# คลาสควบคุม AGV ด้วย Odometry (ไม่ใช้ move_base)
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

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Reached target distance. Final traveled: %.3f m", traveled)
        return True


# ------------------------------------------------------------------
# คลาสควบคุมแขนผ่าน WiFi (เพิ่มความสามารถเลือกหยิบตามรูปร่างและสี)
# ------------------------------------------------------------------
class ArmController:
    def __init__(self, target_shape="Square", target_color=None):
        """
        target_shape: รูปร่างที่ต้องการหยิบ ("Square" หรือ "Hexagon")
        target_color: สีที่ต้องการหยิบ (None = ไม่สนใจสี, หรือ "Red", "Green")
        """
        self.target_shape = target_shape
        self.target_color = target_color
        rospy.loginfo(f"Arm target: shape={target_shape}, color={target_color}")

        try:
            self.mc = MyCobotSocket(ARM_IP, ARM_PORT)
            rospy.loginfo("Arm connected")
        except Exception as e:
            rospy.logerr(f"Arm connection failed: {e}")
            self.mc = None

    def move_to_home(self):
        if not self.mc: return
        rospy.loginfo("Moving arm to home pose...")
        self.mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
        rospy.sleep(5)

    def move_to_calibration(self):
        if not self.mc: return
        rospy.loginfo("Moving arm to calibration pose...")
        self.mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
        rospy.sleep(5)

    def detect_object(self):
        """ตรวจจับวัตถุที่ตรงกับรูปร่างและสีเป้าหมาย (ถ้ากำหนด)"""
        print("กำลังถ่ายภาพ...")
        cap = cv2.VideoCapture(1)  # ปรับ index กล้องตามจริง
        cap.set(3, 640)
        cap.set(4, 480)
        for _ in range(50):
            ret, frame = cap.read()
        if not ret:
            print("Cannot read camera")
            cap.release()
            return None
        cap.release()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in COLOR_RANGES.items():
            # ถ้ากำหนด target_color และไม่ตรงกับสีนี้ ให้ข้ามไป
            if self.target_color is not None and color_name != self.target_color:
                continue

            lower = np.array(lower); upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 250:
                    continue

                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                sides = len(approx)

                shape = ""
                if sides == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    if 0.9 <= float(w)/h <= 1.1:
                        shape = "Square"
                elif sides == 6:
                    shape = "Hexagon"

                if shape == "":
                    continue

                # ตรวจสอบว่ารูปร่างตรงกับเป้าหมายหรือไม่
                if shape != self.target_shape:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cX = x + w//2; cY = y + h//2

                world_x = - (cX - CX) * SCALE
                world_y = (cY - CY) * SCALE

                # (ไม่แสดงภาพ)
                return world_x, world_y, color_name, shape

        rospy.logwarn("No object matching target shape/color detected")
        return None

    def pick_object(self, world_x, world_y):
        target_x = world_x + OFFSET_X
        target_y = world_y + OFFSET_Y

        rospy.loginfo(f"Target base: ({target_x:.1f}, {target_y:.1f})")
        rospy.sleep(5)
        self.mc.send_coords([target_x, target_y, 150.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED); rospy.sleep(5)
        self.mc.set_gripper_state(0, 100); rospy.sleep(3)
        self.mc.send_coords([target_x, target_y, 95.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED); rospy.sleep(4)
        self.mc.set_gripper_state(1, 100); rospy.sleep(3)
        self.mc.send_coords([target_x, target_y, 190.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED); rospy.sleep(4)
        self.mc.send_coords([target_x, target_y, 500.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED); rospy.sleep(4)
        self.mc.send_coords([89.2, -64.2, 194.6, 179.98, 0.99, -46.58], SPEED); rospy.sleep(4)

    def perform_task(self):
        if not self.mc: return False
        self.move_to_calibration()
        result = self.detect_object()
        if result is None:
            return False
        world_x, world_y, color, shape = result
        rospy.loginfo(f"Detected {color} {shape} at ({world_x:.1f}, {world_y:.1f})")
        self.pick_object(world_x, world_y)
        return True


# ------------------------------------------------------------------
# คลาสหลัก รวม AGV + Arm
# ------------------------------------------------------------------
class AGVWithArm:
    def __init__(self):
        rospy.init_node('agv_with_arm_odom')
        self.agv = AGVOdomController()

        # กำหนดเป้าหมายที่ต้องการหยิบ (เปลี่ยนได้ตามต้องการ)
        target_shape = "Square"   # หรือ "Square"
        target_color = "Red"       # หรือ "Green" หรือ None ถ้าไม่สนใจสี
        self.arm = ArmController(target_shape=target_shape, target_color=target_color)

    def run(self):
        # 1. แขนไป home
        self.arm.move_to_home()
        rospy.sleep(1)

        # 2. AGV เดินหน้า 1 เมตร
        rospy.loginfo("AGV moving forward 1 meter...")
        if not self.agv.move_forward(1.0):
            rospy.logerr("AGV movement failed. Exiting.")
            return

        # 3. เมื่อถึงระยะแล้ว ให้แขนทำงาน
        rospy.loginfo("AGV reached target distance. Arm performing task...")
        self.arm.perform_task()

        rospy.loginfo("Mission completed.")


if __name__ == '__main__':
    try:
        node = AGVWithArm()
        node.run()
    except rospy.ROSInterruptException:
        pass
