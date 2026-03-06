#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tf

class NavToAB:
    def __init__(self):
        rospy.init_node('nav_to_ab')

        # กำหนดพิกัดจุด A และ B ใน frame "map" (หน่วยเป็นเมตร)
        # แก้ไขค่าให้ตรงกับตำแหน่งจริงในแผนที่ของคุณ
        self.waypoints = [
            {'name': 'A', 'x': 1.0, 'y': 0.5, 'theta': 0.0},
            {'name': 'B', 'x': 2.5, 'y': 1.8, 'theta': 1.57}  # theta = 90 องศา
        ]

        # เชื่อมต่อกับ move_base action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

    def send_goal(self, x, y, theta, name):
        """ส่งเป้าหมายให้ move_base และรอจนกว่าจะถึง"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # แปลงมุม theta (เรเดียน) เป็น quaternion
        q = tf.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"Sending goal to {name}: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        self.client.send_goal(goal)

        # รอจนกว่าจะถึงเป้าหมาย หรือถูกขัดจังหวะ
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Successfully reached {name}.")
            return True
        else:
            rospy.logwarn(f"Failed to reach {name}.")
            return False

    def run(self):
        """ส่งเป้าหมายไปยัง A และ B ตามลำดับ"""
        for wp in self.waypoints:
            success = self.send_goal(wp['x'], wp['y'], wp['theta'], wp['name'])
            if not success:
                rospy.logerr("Navigation aborted.")
                break
            rospy.sleep(1)  # หยุดพักเล็กน้อยก่อนส่งเป้าหมายถัดไป

        rospy.loginfo("Finished navigating to A and B.")

if __name__ == '__main__':
    try:
        nav = NavToAB()
        nav.run()
    except rospy.ROSInterruptException:
        pass