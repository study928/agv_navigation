#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import actionlib, tf, rospy
import csv, os
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class WpNavi():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while not self.ac.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo('Waiting for the move_base actino server to come up')

        rospy.loginfo('The server comes up')
        self.goal = MoveBaseGoal()
        self.csv_name = "1125.csv"

    def way_point(self):        
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # csv fileを開く
        with open(self.csv_name, "r") as f:
            reader = csv.reader(f) # ファイル読み込み
            header = next(reader) # ファイルのcolumnを読み込まないようにする

            waypoints = [row for row in reader]
            waypoints = [list(map(float,row)) for row in waypoints]

        i = 0
        while not rospy.is_shutdown():
            # ウェイポイントの数になったら終了
            if i == len(waypoints):
                break
            
            self.goal.target_pose.pose.position.x = waypoints[i][1]
            self.goal.target_pose.pose.position.y = waypoints[i][2]
            
            self.goal.target_pose.pose.orientation = Quaternion(0, 0,waypoints[i][6],waypoints[i][7])
            rospy.loginfo('Sending goal: No.' + str(i+1))

            self.ac.send_goal(self.goal)

            succeeded = self.ac.wait_for_result(rospy.Duration())
            
            # 移動中:1, goal:3
            state = self.ac.get_state()

            if succeeded:
                rospy.loginfo('Succeeded: No.' + str(i+1) + '('+str(state)+')')
            else:
                rospy.loginfo('Faild: No.' + str(i+1) +"("+str(state)+")")

            i += 1

    def shutdown(self):
        rospy.loginfo('The robot was terminated')

        self.ac.cancel_goal()

if __name__ == '__main__':
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node('wp_navi')
    w = WpNavi()

    try:
        w.way_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('WP navigation finished')
        
