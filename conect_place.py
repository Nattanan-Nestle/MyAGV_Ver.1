#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import time
from pymycobot import MyCobotSocket   # เปลี่ยนจาก MyCobot280 เป็น MyCobotSocket

# ------------------------------------------------------------------
# กำหนดค่าคงที่ (ต้องปรับตามผล calibration ของคุณ)
# ------------------------------------------------------------------
SPEED = 20

# IP ของแขน (Jetson Nano) – เปลี่ยนให้ตรงกับ IP จริง
ARM_IP = "192.168.137.24"   # ตัวอย่าง
ARM_PORT = 9000
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

    # 1. ขยับไปท่า home
    print("กำลังขยับไปท่า home...")
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)
    time.sleep(5)
    
    print("Open Gripper...")
    mc.set_gripper_state(0, 100) 
    time.sleep(2)
    
    print("Close Gripper...")
    mc.set_gripper_state(1, 100) 
    time.sleep(2)
    
    # 2. ขยับไปท่าที่กล้องมองลง (calibration)
    print("เตรียมวาง...")
    mc.send_angles([0, -20, -65, 0, 0, -45], SPEED)
    time.sleep(5)

    # 6. เคลื่อนที่เหนือวัตถุ (ความสูงปลอดภัย 150 mm)
    print("เคลื่อนที่ไปยังตำแหน่งเหนือเเถ่นวาง...")
    mc.send_coords([257.6, -60.4, 300.0, 177.34, 0.0, -45.28], SPEED)
    time.sleep(5)
    
    mc.send_coords([257.6, -60.4, 95.0, 177.34, 0.0, -45.28], SPEED)  
    time.sleep(5)
    
    print("สั่ง gripper กางออก...")
    mc.set_gripper_state(0, 100)   # 0 = เปิด
    time.sleep(3)

    # 7. อ่านตำแหน่งปัจจุบัน (ถ้าต้องการ)
    # coords = mc.get_coords()
    # print(coords)
    
    # mc.send_coords([257.6, -60.4, 300, 177.34, 0.0, -45.28], SPEED)
    # time.sleep(5)
    mc.send_angles([0, 45, -120, -13, 0, -45], SPEED)

if __name__ == "__main__":
    main()