# MyAGV_Ver.1
โค้ดนี้เป็นการทดสอบเเละใช้งานหุ่นยนต์ MyAGV 2023 Jetson nano ในการทดสอบการเคลื่อนที่ เเละการใช้งาน Lidar 

ขั้นตอนการใช้งาน

ตัวอย่าง คำสั่งเดินหน้า

1. ใช้คำสั่ง  roslaunch myagv_odometry myagv_active.launch
2. เปิดอีกหนึ่ง Terminal เเล้วใช้คำสั่ง cd ~/myagv_ros/src/myagv_navigation/script
3. ใน Terminal myagv_navigation ใช้คำสั่ง python3 move_forward.py 
4. หุ่นยนต์จะเคลื่อนที่ไปตามคำสั่งที่เขียนไว้ใน move_forward.py 

ข้อควรระวัง

ต้องใช้คำสั่ง roslaunch myagv_odometry myagv_active.launch ก่อนเสมอเพราะจะใช้ค่า odomety จาก Lidar IMU Encoder ของหุ่นยนต์  

