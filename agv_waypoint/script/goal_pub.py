#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, tf, actionlib, math
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
    [(2.30402374268,-2.15009021759,0.0),(0.0,0.0,-0.821138468057,0.570729021758)],
    [(2.93424391747,0.420744687319,0.0),(0.0,0.0,0.653460739151,0.756960410054)],
    [(-0.555642783642,0.699080049992,0.0),(0.0,0.0,-0.183689945362,0.982984233837)],
    [(1.89684021473,-0.181654199958,0.0),(0.0,0.0,0.998276122303,0.0586922792232)]
]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == '__main__':
    rospy.init_node('patrol')
    listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(4.0))
    
    while True:
        i = 0
        for pose in waypoints:
            goal = goal_pose(pose)
            client.send_goal(goal)
            print(goal.target_pose.pose.position.z + i)
                
            while True:
                now = rospy.Time.now()
                listener.waitForTransform('map', 'base_link', now, rospy.Duration(4.0))
                position, quaternion = listener.lookupTransform('map', 'base_link', now)

                if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2) <= 0.5):
                    print ("next goal!!")
                    break
                elif goal.target_pose.pose.position.x == 1.89684021473:
                    break
                else:
                    rospy.sleep(0.5)

            i += 1

        if goal.target_pose.pose.position.x == 1.89684021473:
            print('finish!!')
            break