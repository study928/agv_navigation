#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionGoal

def marker_pub():
    rospy.loginfo("Start")
    pub = rospy.Publisher('pose_waypoint', MarkerArray, queue_size=10)

    rate = rospy.Rate(100)
    csv_name = 'pose.csv' # csvファイル名
    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        # csvファイル読み込み
        with open(csv_name, 'r') as f:
            counter = 4
            reader = csv.reader(f)
            # 999だったらなにもしない
            for row in reader:
                if row[0] == "999":
                    pass

                marker_data = Marker()
                marker_data.header.frame_id = 'map'
                marker_data.header.stamp = rospy.Time.now()

                marker_data.ns = 'basic_shapes'
                marker_data.id = counter

                marker_data.action = Marker.ADD

                # marker position
                marker_data.pose.position.x = map(float,row)[0]
                marker_data.pose.position.y = map(float,row)[1]
                marker_data.pose.position.z = map(float,row)[2]
                #marker orientation
                marker_data.pose.orientation.x=map(float,row)[3]
                marker_data.pose.orientation.y=map(float,row)[4]
                marker_data.pose.orientation.z=map(float,row)[5]
                marker_data.pose.orientation.w=map(float,row)[6]

                marker_data.color.r = 0.0
                marker_data.color.g = 1.0
                marker_data.color.b = 0.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 0.2
                marker_data.scale.y = 0.1
                marker_data.scale.z = 0.05

                marker_data.lifetime = rospy.Duration()
                # marker typeを矢印に設定
                marker_data.type = 0

                marker_array.markers.append(marker_data)

                pub.publish(marker_array)
                counter +=1
        
        rate.sleep()
        
if __name__ == "__main__":
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node('pose_waypoint')
    try:
        marker_pub()
        rospy.spin()
    except rospy.ROSInterruptException : 
        rospy.logdebug("Can't read csv file")  