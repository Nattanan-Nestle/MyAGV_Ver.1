# MyAGV_Ver.1
โค้ดนี้เป็นการทดสอบเเละใช้งานหุ่นยนต์ MyAGV 2023 Jetson nano ในการทดสอบการเคลื่อนที่ เเละการใช้งาน Lidar 

โหลดโปรเเกรม MobaXtrem https://mobaxterm.mobatek.net/download.html

1. เชื่อมต่อจอ เมาส์ คีย์บอร์ดให้เรียบร้อย
2. เมื่อเชื่อมต่อเสร็จให้เชื่อมต่อ Internet ให้เรียบร้อย
3. ให้เช็ค IP ของเเขนให้เรียบร้อย ด้วยการเปิด Terminal เเล้วพิมพ์ ip addr
4. เข้าไปที่ cd ~/myagv_ros/src/myagv_navigation/script
5. ใช้คำสั่ง git clone https://github.com/Nattanan-Nestle/MyAGV_Ver.1.git


ตัวอย่าง คำสั่งเดินหน้า
1. ใช้คำสั่ง  roslaunch myagv_odometry myagv_active.launch
2. เปิดอีกหนึ่ง Terminal เเล้วใช้คำสั่ง cd ~/myagv_ros/src/myagv_navigation/script/MyAGV_Ver.1
3. ใน Terminal /myagv_ros/src/myagv_navigation/script/MyAGV_Ver.1 ใช้คำสั่ง python3 move_forward.py 
4. หุ่นยนต์จะเคลื่อนที่ไปตามคำสั่งที่เขียนไว้ใน move_forward.py 

ข้อควรระวัง

ต้องใช้คำสั่ง roslaunch myagv_odometry myagv_active.launch ก่อนเสมอเพราะจะใช้ค่า odomety จาก Lidar IMU Encoder ของหุ่นยนต์  

