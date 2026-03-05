#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveForwardDistance:
    def __init__(self):
        rospy.init_node('move_forward_distance')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = None
        self.start_y = None
        self.target_distance = 0.0
        self.moving = False

    def odom_callback(self, msg):
        # อ่านตำแหน่งปัจจุบันจาก odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def move_forward(self, distance_meters, speed=0.2):
        """
        สั่งให้หุ่นยนต์เคลื่อนที่ไปข้างหน้าตามระยะทาง (เมตร)
        speed: ความเร็วเชิงเส้น (เมตร/วินาที) ค่าเริ่มต้น 0.2 m/s
        """
        rospy.sleep(0.5)  # รอให้ odometry พร้อม
        
        # บันทึกตำแหน่งเริ่มต้น
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.target_distance = distance_meters
        self.moving = True

        twist = Twist()
        twist.linear.x = speed  # ความเร็วคงที่
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and self.moving:
            # คำนวณระยะทางที่เคลื่อนที่ไปแล้ว (Euclidean distance)
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            traveled = math.sqrt(dx*dx + dy*dy)
            
            rospy.loginfo("เคลื่อนที่ไปแล้ว: %.2f เมตร", traveled)
            
            if traveled >= self.target_distance:
                self.moving = False
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("ถึงเป้าหมาย %.2f เมตร", self.target_distance)
                break
            
            self.cmd_pub.publish(twist)
            rate.sleep()
        
        # หยุดหุ่นยนต์เมื่อจบ (เผื่อหลุด loop)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        mover = MoveForwardDistance()
        rospy.sleep(1)  # รอให้ระบบพร้อม
        
        # ตัวอย่าง: เคลื่อนที่ไปข้างหน้า 1.0 เมตร ด้วยความเร็ว 0.2 m/s
        mover.move_forward(1.0, speed=0.2)
        
        # ถ้าต้องการ 1.2 เมตร ก็เปลี่ยนเป็น mover.move_forward(1.2, 0.2)
        
    except rospy.ROSInterruptException:
        pass