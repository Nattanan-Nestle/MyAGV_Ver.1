#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
from pymycobot import MyCobotSocket   # เปลี่ยนจาก MyCobot280 เป็น MyCobotSocket

# ------------------------------------------------------------------
# กำหนดค่าคงที่ (ต้องปรับตามผล calibration ของคุณ)
# ------------------------------------------------------------------
SCALE = 0.4050           # mm/pixel
CX = 320
CY = 240
OFFSET_X = 270           # mm offset
OFFSET_Y = -59
FIXED_RX = -178.69
FIXED_RY = -0.29
FIXED_RZ = -45.26

COLOR_RANGES = {
    "Red":    [(168, 151, 185),  (175, 255, 255)],
    "Green":  [(75, 194, 157),   (83, 255, 214)],
}

SPEED = 20

# IP ของแขน (Jetson Nano) – เปลี่ยนให้ตรงกับ IP จริง
ARM_IP = "192.168.137.119"   # ตัวอย่าง
ARM_PORT = 9000

# ------------------------------------------------------------------
# ฟังก์ชันตรวจจับวัตถุ (เหมือนเดิม)
# ------------------------------------------------------------------
def detect_object(frame, scale):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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

            return world_x, world_y, color_name, shape, cX, cY

    return None

# ------------------------------------------------------------------
# ฟังก์ชันหลัก (ใช้ MyCobotSocket)
# ------------------------------------------------------------------
def main():
    print("เชื่อมต่อกับแขนผ่าน WiFi...")
    try:
        mc = MyCobotSocket(ARM_IP, ARM_PORT)
        print("เชื่อมต่อสำเร็จ!")
    except Exception as e:
        print(f"เชื่อมต่อล้มเหลว: {e}")
        return

    # (ไม่จำเป็นต้อง sleep 2 วินาที เพราะ MyCobotSocket ไม่ต้องเปิด serial)

    # 1. ขยับไปท่า home
    print("กำลังขยับไปท่า home...")
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)

    # 2. ขยับไปท่าที่กล้องมองลง (calibration)
    print("กำลังขยับไปท่า calibration...")
    mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
    time.sleep(5)

    # 3. ถ่ายภาพจากกล้องของคอมพิวเตอร์ (ไม่ใช่กล้องบนแขน)
    print("กำลังถ่ายภาพ...")
    cap = cv2.VideoCapture(0)          # กล้อง USB ที่ต่อกับ Windows
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # รอให้กล้องปรับแสง
    for _ in range(100):
        ret, frame = cap.read()
        if not ret:
            print("ไม่สามารถอ่านภาพจากกล้องได้")
            cap.release()
            return

    cap.release()

    # 4. ตรวจจับวัตถุ
    result = detect_object(frame, SCALE)
    if result is None:
        print("ไม่พบวัตถุในภาพ")
        print("กำลังกลับ HOME")
        mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
        return
    world_x, world_y, color_name, shape, cX, cY = result
    print(f"พบวัตถุ: {color_name} {shape}")
    print(f"พิกัดจากกล้อง (cam): ({world_x:.1f}, {world_y:.1f}) mm")

    # 5. คำนวณตำแหน่งเป้าหมายในฐานแขน
    target_x = world_x + OFFSET_X
    target_y = world_y + OFFSET_Y
    print(f"ตำแหน่งเป้าหมาย (base): ({target_x:.1f}, {target_y:.1f}) mm")

    # (Optional) ตรวจสอบ workspace
    if target_x < 0 or target_x > 400 or target_y < -300 or target_y > 300:
        print("Warning: ตำแหน่งเป้าหมายอาจอยู่นอก workspace")

    # 6. เคลื่อนที่เหนือวัตถุ (ความสูงปลอดภัย 150 mm)
    print("เคลื่อนที่ไปยังตำแหน่งเหนือวัตถุ...")
    mc.send_coords([target_x, target_y, 150.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(5)   # เผื่อเวลาเดินทางไกล

    print("สั่ง gripper กางออก...")
    mc.set_gripper_state(0, 100)   # 0 = เปิด
    time.sleep(3)

    # 7. อ่านตำแหน่งปัจจุบัน (ถ้าต้องการ)
    current = mc.get_coords()
    if current:
        print(f"ตำแหน่งปัจจุบัน: x={current[0]:.1f}, y={current[1]:.1f}, z={current[2]:.1f}")

    # 8. ลด Z ลงไปหยิบ
    print("ลด Z ลงไปหยิบ...")
    mc.send_coords([target_x, target_y, 95.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)

    # 9. ปิด gripper หนีบ
    print("สั่ง gripper หนีบ...")
    mc.set_gripper_state(1, 100)   # 1 = ปิด
    time.sleep(3)

    # 10. ยกขึ้นและย้ายไปวาง (ตัวอย่าง)
    print("ยกขึ้นแล้วไปวาง...")
    mc.send_coords([target_x, target_y, 190.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)
    mc.send_coords([target_x, target_y, 500.0, FIXED_RX, FIXED_RY, FIXED_RZ], SPEED)
    time.sleep(4)
    mc.send_coords([89.2, -64.2, 194.6, 179.98, 0.99, -46.58], SPEED)
    print("เสร็จสิ้น")

    # แสดงภาพที่ตรวจจับ (optional)
    cv2.circle(frame, (cX, cY), 5, (255,255,255), -1)
    cv2.putText(frame, f"{color_name} {shape}", (cX+40, cY-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
    cv2.imshow("Detection", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()