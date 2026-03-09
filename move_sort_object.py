#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import tf.transformations as tf
from pymycobot import MyCobotSocket

# ------------------------------------------------------------------
# ค่าคงที่ (ปรับตามการ calibrate ของคุณ)
# ------------------------------------------------------------------
SCALE = 0.4030
CX = 320
CY = 240
OFFSET_X = 265
OFFSET_Y = -59
FIXED_RX = -178.69
FIXED_RY = -0.29
FIXED_RZ = -45.26

COLOR_RANGES = {
    "Red":    [(169, 119, 139),  (179, 255, 255)],
    "Green":  [(68, 87, 61),   (81, 255, 255)], 
}

TARGET_COLOR = "Red"
TARGET_SHAPE = "Hexagon"

SPEED = 20
ARM_IP = "192.168.137.119"
ARM_PORT = 9000

# ตำแหน่งวางวัตถุ (บน AGV หรือจุดทิ้ง)
PLACE_X, PLACE_Y, PLACE_Z = 89.2, -64.2, 194.6

#ค่าควบคุม AGV
LINEAR_SPEED = 0.2        # m/s ปรับความเร็ววิ่งตรง
ANGULAR_SPEED = 0.33       # rad/s ความเร็วในการเลี้ยว 
POS_TOLERANCE = 0.05      # เมตร ( tolerance การขับถึงจุด)
ANGLE_TOLERANCE = 0.01    # เรเดียน 

# ------------------------------------------------------------------
# คลาส AGVMover (ใช้ IMU สำหรับหมุน, odometry สำหรับระยะทาง)
# ------------------------------------------------------------------
class AGVMover:
    def __init__(self):
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.imu_yaw = 0.0

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.loginfo("AGVModer initialized")

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        q = msg.orientation
        (_, _, self.imu_yaw) = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def get_position(self):
        return self.odom_x, self.odom_y

    def get_yaw(self):
        return self.imu_yaw

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def rotate_relative(self, angle_deg):
        """หมุนตามเข็มนาฬิกา (+angle_deg) ใช้ IMU เป็น feedback"""
        target_yaw = self.imu_yaw + np.radians(angle_deg)
        # ปรับให้อยู่ในช่วง -pi ถึง pi
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))

        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            current_yaw = self.imu_yaw
            error = target_yaw - current_yaw
            error = np.arctan2(np.sin(error), np.cos(error))

            if abs(error) < ANGLE_TOLERANCE:
                break

            twist.angular.z = np.clip(1.45 * error, -ANGULAR_SPEED, ANGULAR_SPEED)
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop()
        rospy.loginfo(f"Rotated {angle_deg} deg, final yaw: {self.imu_yaw:.2f} rad")

    def drive_forward(self, distance_m):
        """ขับตรงไปข้างหน้าเป็นระยะทาง distance_m (เมตร) โดยใช้ odometry"""
        start_x, start_y = self.get_position()
        traveled = 0.0
        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown() and traveled < distance_m:
            # รักษาทิศทางด้วย IMU (เพื่อไม่ให้เบี่ยง)
            current_yaw = self.imu_yaw
            # คำนวณระยะทางที่เคลื่อนที่ไปแล้ว
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            traveled = np.hypot(dx, dy)

            # ขับตรง
            twist.linear.x = LINEAR_SPEED
            # แก้ไขการหมุนเล็กน้อยตาม error ของมุม (เทียบกับทิศทางเริ่มต้น)
            # (ถ้าต้องการให้ตรงสนิทอาจเก็บ target_yaw ตอนเริ่ม)
            # แต่ในที่นี้เราใช้ IMU รักษาทิศทางปัจจุบันไว้
            twist.angular.z = 0.0  # หรือปรับตาม error ถ้ามีการเบี่ยงเบน
            self.cmd_pub.publish(twist)
            rate.sleep()

        self.stop()
        rospy.loginfo(f"Driven forward {traveled:.2f} m")


# ------------------------------------------------------------------
# ฟังก์ชันตรวจจับวัตถุ (คืนค่าเป็นลิสต์ของวัตถุทั้งหมดที่พบ)
# ------------------------------------------------------------------
def detect_objects(frame, scale):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    objects = []

    for color_name, (lower, upper) in COLOR_RANGES.items():
        lower = np.array(lower)
        upper = np.array(upper)
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
                ratio = float(w) / h
                if 0.9 <= ratio <= 1.1:
                    shape = "Square"
            elif sides == 6:
                shape = "Hexagon"

            if shape == "":
                continue

            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                x, y, w, h = cv2.boundingRect(cnt)
                cX = x + w//2
                cY = y + h//2

            world_x = - (cX - CX) * scale
            world_y = (cY - CY) * scale

            objects.append((world_x, world_y, color_name, shape, cX, cY))

    return objects
# ------------------------------------------------------------------
# ฟังก์ชันให้แขนทำงาน ณ จุดปัจจุบัน (scan, pick, place)
# คืนค่า True ถ้าหยิบวัตถุสำเร็จ, False ถ้าไม่พบ
# ------------------------------------------------------------------
def scan_and_pick(mc):
    # 1. ขยับ calibration pose
    print(" calibrating...")
    mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
    time.sleep(5)

    # 2. ถ่ายภาพ
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    for i in range(100):
        ret, frame = cap.read()
        if not ret:
            print("ไม่สามารถอ่านภาพจากกล้องได้")
            cap.release()
            return

    cap.release()

    # 3. ตรวจจับ
    objects = detect_objects(frame, SCALE)
    if not objects:
        print(" no objects found")
        return False

    # แสดงรายการ
    for obj in objects:
        wx, wy, col, shp, _, _ = obj
        print(f"  {col} {shp} at ({wx:.1f}, {wy:.1f}) mm")

    # 4. เลือกเป้าหมาย
    target = None
    for obj in objects:
        wx, wy, col, shp, cx, cy = obj
        if col == TARGET_COLOR and shp == TARGET_SHAPE:
            target = obj
            break

    if target is None:
        print(f" no target {TARGET_COLOR} {TARGET_SHAPE}")
        return False

    world_x, world_y, color_name, shape, cX, cY = target
    print(f"target: {color_name} {shape} at cam ({world_x:.1f}, {world_y:.1f}) mm")

    # 5. คำนวณพิกัดฐานแขน
    target_x = world_x + OFFSET_X
    target_y = world_y + OFFSET_Y
    print(f"base target: ({target_x:.1f}, {target_y:.1f}) mm")

    # 6. เคลื่อนที่เหนือวัตถุ
    mc.send_coords([target_x, target_y, 150.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)

    # 7. เปิด gripper
    mc.set_gripper_state(0, 100)
    time.sleep(3)

    # 8. ลด Z ลง
    pick_z = 95.0
    mc.send_coords([target_x, target_y, pick_z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 9. ปิด gripper
    mc.set_gripper_state(1, 100)
    time.sleep(3)

    # 10. ยกขึ้น
    lift_z = pick_z + 100
    mc.send_coords([target_x, target_y, lift_z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 11. ย้ายไปวาง
    mc.send_coords([target_x, target_y, lift_z+50, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)
    mc.send_coords([PLACE_X, PLACE_Y, PLACE_Z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)
    
    mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
    
    # 14. พับแขนกลับ home
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)

    return True

# ------------------------------------------------------------------
# ฟังก์ชันหลัก
# ------------------------------------------------------------------
def main():
    rospy.init_node('agv_arm_sequence')
    mover = AGVMover()

    # เชื่อมต่อแขน
    try:
        mc = MyCobotSocket(ARM_IP, ARM_PORT)
        print("Connected to arm")
    except Exception as e:
        print(f"Arm connection failed: {e}")
        return

    # เริ่มต้น: พับแขน (home)
    print("Folding arm...")
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)

    # ----------------------------------------------------------
    # ขั้นที่ 1: เดินหน้า 1 เมตร ไปจุด A
    # ----------------------------------------------------------
    print("Step 1: Drive forward 1 m to point A")
    mover.drive_forward(1.0)

    # ----------------------------------------------------------
    # ขั้นที่ 2: สแกนที่จุด A
    # ----------------------------------------------------------
    print("Step 2: Scanning at point A")
    found = scan_and_pick(mc)
    if found:
        print("Mission complete.")
        return

    # ----------------------------------------------------------
    # ขั้นที่ 3: ไม่เจอ -> หมุนขวา 90 องศา
    # ----------------------------------------------------------
    print("Step 3: No target, rotate right 90 deg")
    # พับแขนก่อนเคลื่อนที่
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)
    mover.rotate_relative(90)   # หมุนตามเข็ม 90 องศา

    # ----------------------------------------------------------
    # ขั้นที่ 4: เดินหน้า 0.5 เมตร ไปจุด B
    # ----------------------------------------------------------
    print("Step 4: Drive forward 0.5 m to point B")
    mover.drive_forward(0.5)

    # ----------------------------------------------------------
    # ขั้นที่ 5: หมุนซ้าย 90 องศา (เพื่อกลับทิศเดิม) แล้วสแกน
    # ----------------------------------------------------------
    print("Step 5: Rotate left 90 deg to face original direction")
    mover.rotate_relative(-90)   # หมุนทวนเข็ม 90

    print("Step 6: Scanning at point B")
    found = scan_and_pick(mc)
    if found:
        print("Mission complete.")
    else:
        print("No object found at point B. Mission finished.")

    # พับแขน (ถ้ายังไม่พับ)
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)

if __name__ == "__main__":
    main()
