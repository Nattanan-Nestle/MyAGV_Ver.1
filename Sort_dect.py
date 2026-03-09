#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
from pymycobot import MyCobotSocket

# ------------------------------------------------------------------
# กำหนดค่าคงที่ (ต้องปรับตามผล calibration ของคุณ)
# ------------------------------------------------------------------
SCALE = 0.3923          # mm/pixel
CX = 320
CY = 240
OFFSET_X = 265           # mm offset
OFFSET_Y = -59
FIXED_RX = -178.69
FIXED_RY = -0.29
FIXED_RZ = -45.26

# ช่วงสี HSV
COLOR_RANGES = {
    "Red":    [(168, 147, 149),  (179, 255, 255)],
    "Green":  [(75, 194, 157),   (83, 255, 214)],
}

# กำหนดเป้าหมายที่ต้องการ (เปลี่ยนได้ตามต้องการ)
TARGET_COLOR = "Red"      # สีที่ต้องการ
TARGET_SHAPE = "Hexagon"   # รูปร่างที่ต้องการ เช่น "Square", "Hexagon" 

SPEED = 20

# IP ของแขน (Jetson Nano)
ARM_IP = "192.168.137.24"
ARM_PORT = 9000

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
# ฟังก์ชันหลัก
# ------------------------------------------------------------------
def main():
    print("เชื่อมต่อกับแขนผ่าน WiFi...")
    try:
        mc = MyCobotSocket(ARM_IP, ARM_PORT)
        print("เชื่อมต่อสำเร็จ!")
    except Exception as e:
        print(f"เชื่อมต่อล้มเหลว: {e}")
        return

    # 1. ขยับไปท่า home
    print("กำลังขยับไปท่า home...")
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)

    # 2. ขยับไปท่ากล้องมองลง
    print("กำลังขยับไปท่า calibration...")
    mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
    time.sleep(5)

    # 3. ถ่ายภาพจากกล้องคอมพิวเตอร์
    print("กำลังถ่ายภาพ...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # อ่านภาพ
    ret, frame = cap.read()
    if not ret:
        print("ไม่สามารถอ่านภาพจากกล้องได้")
        cap.release()
        return
    cap.release()

    # 4. ตรวจจับวัตถุทั้งหมด
    objects = detect_objects(frame, SCALE)
    if not objects:
        print("ไม่พบวัตถุใด ๆ ในภาพ")
        print("กำลังกลับ HOME")
        mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
        return

    # แสดงรายการวัตถุที่พบ
    print("วัตถุที่พบทั้งหมด:")
    for obj in objects:
        wx, wy, col, shp, _, _ = obj
        print(f"  {col} {shp} ที่ ({wx:.1f}, {wy:.1f}) mm")

    # 5. เลือกวัตถุที่ตรงกับเป้าหมาย (สีและรูปร่าง)
    target_obj = None
    for obj in objects:
        wx, wy, col, shp, cx, cy = obj
        if col == TARGET_COLOR and shp == TARGET_SHAPE:
            target_obj = obj
            break

    if target_obj is None:
        print(f"ไม่พบวัตถุที่เป็น {TARGET_COLOR} {TARGET_SHAPE}")
        print("กำลังกลับ HOME")
        mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
        return

    world_x, world_y, color_name, shape, cX, cY = target_obj
    print(f"เลือกวัตถุเป้าหมาย: {color_name} {shape}")
    print(f"พิกัดจากกล้อง (cam): ({world_x:.1f}, {world_y:.1f}) mm")

    # 6. คำนวณตำแหน่งฐานแขน
    target_x = world_x + OFFSET_X
    target_y = world_y + OFFSET_Y
    print(f"ตำแหน่งเป้าหมาย (base): ({target_x:.1f}, {target_y:.1f}) mm")

    # ตรวจสอบขอบเขตการทำงาน
    if target_x < 0 or target_x > 400 or target_y < -300 or target_y > 300:
        print("Warning: ตำแหน่งเป้าหมายอาจอยู่นอก workspace")

    # 7. เคลื่อนที่เหนือวัตถุ
    print("เคลื่อนที่ไปยังตำแหน่งเหนือวัตถุ...")
    mc.send_coords([target_x, target_y, 150.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)

    # 8. เปิดกริปเปอร์ก่อนลงไปหยิบ
    print("สั่ง gripper กางออก...")
    mc.set_gripper_state(0, 100)
    time.sleep(3)

    # 9. ลด Z ลงไปหยิบ
    pick_z = 95.0
    print(f"ลด Z ลงไปที่ {pick_z} mm...")
    mc.send_coords([target_x, target_y, pick_z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 10. ปิดกริปเปอร์หนีบ
    print("สั่ง gripper หนีบ...")
    mc.set_gripper_state(1, 100)
    time.sleep(3)
    
    # 11. ยกขึ้นเล็กน้อยก่อนย้าย
    lift_before_move = 100
    lift_z = pick_z + lift_before_move
    print(f"ยก Z ขึ้นไปที่ {lift_z:.1f} mm...")
    mc.send_coords([target_x, target_y, lift_z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 12. ย้ายไปตำแหน่งวาง (ตัวอย่าง)
    place_x, place_y, place_z = 89.2, -64.2, 194.6
    print("เคลื่อนที่ไปยังตำแหน่งวาง...")
    # บินสูงก่อน
    mc.send_coords([target_x, target_y, lift_z + 50, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)
    
    # 14. ยก Z ขึ้นจากจุดวางก่อนกลับ home (ใช้ place_z เป็นฐาน)
    lift_after_place = 150
    new_place_z = place_z + lift_after_place
    print(f"ยก Z ขึ้นจากจุดวางไปที่ {new_place_z:.1f} mm...")
    mc.send_coords([place_x, place_y, new_place_z, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 15. กลับสู่ท่า home
    print("กลับสู่ท่า home")
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)

if __name__ == "__main__":
    main()