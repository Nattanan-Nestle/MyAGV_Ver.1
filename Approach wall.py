#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ApproachWall:
    def __init__(self):
        rospy.init_node('approach_wall')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.target_min = 0.15  # 150 mm
        self.target_max = 0.20  # 200 mm
        self.speed = 0.1  # m/s
        self.angle_range = 20  # องศา ที่ถือว่าเป็นด้านหน้า (รวม center)
        self.scan_received = False
        self.current_distance = float('inf')
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
    def scan_callback(self, msg):
        # หาระยะทางไปผนังด้านหน้า
        # สมมติว่า /scan มีมุม 0 ตรงกลาง (หรือตาม configuration ของ lidar)
        # หา index ของมุมที่ใกล้ 0 องศา
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # คำนวณ index ที่ตรงกับมุม 0 (หรือใกล้ที่สุด)
        # โดยทั่วไป lidar จะมี 0 อยู่ตรงกลาง แต่อาจจะต้องปรับตาม
        # สมมติว่ามุม 0 อยู่ตรงกลาง ดังนั้น index = int((0 - angle_min) / angle_increment)
        # หรือหา index ช่วงมุมที่สนใจ เช่น -10 ถึง +10 องศา
        
        # เปลี่ยนองศาเป็นเรเดียน
        target_angle = 0.0  # เรเดียน
        angle_tolerance = np.radians(self.angle_range / 2.0)  # +/- range
        
        distances = []
        for i in range(len(msg.ranges)):
            angle = angle_min + i * angle_increment
            if abs(angle - target_angle) <= angle_tolerance:
                d = msg.ranges[i]
                if d > msg.range_min and d < msg.range_max and not np.isinf(d) and not np.isnan(d):
                    distances.append(d)
        
        if distances:
            # ใช้ค่าที่น้อยที่สุด (ใกล้ที่สุด) เพื่อความปลอดภัย? หรือค่าเฉลี่ย?
            self.current_distance = min(distances)  # ใช้ค่าที่ใกล้ที่สุด
            self.scan_received = True
        else:
            self.current_distance = float('inf')
            self.scan_received = False
            
    def run(self):
        rospy.loginfo("Waiting for laser scan data...")
        while not rospy.is_shutdown() and not self.scan_received:
            self.rate.sleep()
        rospy.loginfo("Laser scan data received. Current distance: %.3f m", self.current_distance)
        
        twist = Twist()
        
        while not rospy.is_shutdown():
            if not self.scan_received:
                rospy.logwarn("No laser data, stopping")
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                continue
            
            d = self.current_distance
            rospy.loginfo("Distance to wall: %.3f m", d)
            
            if d >= self.target_min and d <= self.target_max:
                rospy.loginfo("Target reached. Stopping.")
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                break
            elif d > self.target_max:
                # อยู่ไกลเกินไป เดินหน้า
                twist.linear.x = self.speed
            else:
                # อยู่ใกล้เกินไป ถอยหลัง (แต่โจทย์เริ่มจากไกล ดังนั้นไม่น่าเจอ)
                twist.linear.x = -self.speed/2  # ถอยช้าๆ
            
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        
        rospy.loginfo("Approach wall completed.")

if __name__ == '__main__':
    try:
        aw = ApproachWall()
        aw.run()
    except rospy.ROSInterruptException:
        pass