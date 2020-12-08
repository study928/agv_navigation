#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
from visualization_msgs.msg import Marker, MarkerArray

def marker_pub():
    rospy.loginfo("Start")
    waypoint_pub = rospy.Publisher('waypoint_marker', MarkerArray, queue_size=10)

    rate = rospy.Rate(10)
    csv_name = rospy.get_param("~csv_name","1203.csv") # csvファイル名 rosparam

    while not rospy.is_shutdown():
        marker_array = MarkerArray()
        with open(csv_name, "r") as f:
            counter = 4
            reader = csv.reader(f)

            for row in reader:
                if row[1] == "1":
                    marker_data = Marker()
                    marker_data.header.frame_id = 'map'
                    marker_data.header.stamp = rospy.Time.now()

                    marker_data.ns = 'basic_shapes'
                    marker_data.id = counter

                    marker_data.action = Marker.ADD

                    marker_data.pose.position.x = map(float,row)[2]
                    marker_data.pose.position.y = map(float,row)[3]
                    marker_data.pose.position.z = map(float,row)[4]
                    #marker orientation
                    marker_data.pose.orientation.x=map(float,row)[5]
                    marker_data.pose.orientation.y=map(float,row)[6]
                    marker_data.pose.orientation.z=map(float,row)[7]
                    marker_data.pose.orientation.w=map(float,row)[8]

                    marker_data.color.r = 1.0
                    marker_data.color.g = 0.0
                    marker_data.color.b = 0.0
                    marker_data.color.a = 1.0
                    marker_data.scale.x = 1.0
                    marker_data.scale.y = 0.1
                    marker_data.scale.z = 0.1

                    marker_data.lifetime = rospy.Duration()
                    # marker typeを矢印に設定
                    marker_data.type = 0
                    marker_array.markers.append(marker_data)

                    waypoint_pub.publish(marker_array)
                    counter +=1


                    # Mark num
                    marker_data = Marker()
                    marker_data.header.frame_id = 'map'
                    marker_data.header.stamp = rospy.Time.now()

                    marker_data.ns = 'basic_shapes'
                    marker_data.id = counter

                    marker_data.action = Marker.ADD

                    marker_data.pose.position.x = map(float,row)[2]
                    marker_data.pose.position.y = map(float,row)[3]
                    marker_data.pose.position.z = map(float,row)[4]
                    #marker orientation
                    marker_data.pose.orientation.x=map(float,row)[5]
                    marker_data.pose.orientation.y=map(float,row)[6]
                    marker_data.pose.orientation.z=map(float,row)[7]
                    marker_data.pose.orientation.w=map(float,row)[8]

                    marker_data.color.r = 0.0
                    marker_data.color.g = 0.0
                    marker_data.color.b = 0.0
                    marker_data.color.a = 1.0
                    marker_data.scale.z = 1.0

                    marker_data.lifetime = rospy.Duration()

                    marker_data.type = Marker.TEXT_VIEW_FACING
                    marker_data.text = str(int(map(float,row)[0]))

                    # csvファイルのnumをmapに表示
                    marker_array.markers.append(marker_data)

                    waypoint_pub.publish(marker_array)
                    counter +=1

                else :
                    # Marker amcl_pose
                    marker_data = Marker()
                    marker_data.header.frame_id = 'map'
                    marker_data.header.stamp = rospy.Time.now()

                    marker_data.ns = 'basic_shapes'
                    marker_data.id = counter

                    marker_data.action = Marker.ADD

                    # marker position
                    marker_data.pose.position.x = map(float,row)[2]
                    marker_data.pose.position.y = map(float,row)[3]
                    marker_data.pose.position.z = map(float,row)[4]
                    #marker orientation
                    marker_data.pose.orientation.x=map(float,row)[5]
                    marker_data.pose.orientation.y=map(float,row)[6]
                    marker_data.pose.orientation.z=map(float,row)[7]
                    marker_data.pose.orientation.w=map(float,row)[8]

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

                    waypoint_pub.publish(marker_array)
                    counter +=1
        
        rate.sleep()
        
if __name__ == "__main__":
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node('waypoint_marker')
    try:
        marker_pub()
        rospy.spin()
    except rospy.ROSInterruptException : 
        rospy.logdebug("Can't read csv file") 