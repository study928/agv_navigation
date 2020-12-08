#!/usr/bin/env python                                                            
# -*- coding: utf-8 -*-                                                          

import actionlib,rospy, tf
import csv, os
from math import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import String

class WpNavi():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.led_sub = rospy.Subscriber("/cmd_led", String, self.led_callback)
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # actionlib server起動
        while not self.ac.wait_for_server(rospy.Duration(5.0)):
            rospy.loginfo('Waiting for the move_base actino server to come up')
        rospy.loginfo('The server comes up')

        self.goal = MoveBaseGoal()
        self.csv_name = rospy.get_param("~csv_name","1203.csv")# csvファイル名　目標地とロボットの軌跡が記録されている
        self.led = "0"

    # ロボットの現在位置を取得
    def pose_callback(self, pose_msg):
        self.x = pose_msg.pose.pose.position.x
        self.y = pose_msg.pose.pose.position.y

    def led_callback(self, led_msg):
        self.led = led_msg.data

    def way_point(self):     
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # csv fileを開く
        with open(self.csv_name, "r") as f:
            reader = csv.reader(f) # ファイル読み込み
            waypoints = [row for row in reader]
            waypoints = [list(map(float,row)) for row in waypoints] # float型に変換
            waypoints_num = len(waypoints) # ウェイポイント数

        i = 0
        last_distance = 0.0
        begin_navigation = rospy.Time.now() # 新しいナビゲーションを設定した時間
        verbose_start = rospy.Time.now()
        while not rospy.is_shutdown():
            # ウェイポイント数に達したら,i=0として最初からループ
            if i == waypoints_num:
                rospy.loginfo("Loop!!")
                i = 0
            # ウェイポイントの位置
            self.goal.target_pose.pose.position.x = waypoints[i][2]
            self.goal.target_pose.pose.position.y = waypoints[i][3]
            # ウェイポイントの向き
            self.goal.target_pose.pose.orientation = Quaternion(0, 0,waypoints[i][7],waypoints[i][8])
            rospy.loginfo('Sending goal: No.' + str(i+1))
            # serverにウェイポイントの情報を送る
            self.ac.send_goal(self.goal)
            
            
            while not rospy.is_shutdown():
                # flagの判定 ロボットの軌跡:0.0, 目標値:1.0
                if waypoints[i][1] == 1.0:
                    # severに送った結果が返ってくるのを待つ
                    # succeededを記述するとdistanceの計算が1回だけ行われるようになり,目標地まで到達できる
                    succeeded = self.ac.wait_for_result(rospy.Duration())
                    if succeeded:
                        rospy.loginfo("Goal!!")
                        # 目標地で実行したい処理が終わるまで保留
                        while not rospy.is_shutdown():
                            # 処理が終了したら次のウェイポイントへ
                            if self.led == "1":
                                rospy.loginfo("The work is done!")
                                break
                            else :
                                rospy.sleep(2.0)
                                rospy.loginfo("Now working")
                                
                        rospy.sleep(5.0)
                        rospy.loginfo("Next!!")
                        break

                # ロボットの現在位置とウェイポイント間の距離
                distance = sqrt(pow((self.goal.target_pose.pose.position.x-self.x), 2) + 
                         pow((self.goal.target_pose.pose.position.y-self.y), 2))
                # ロボットが進んだ距離
                delta_distance = abs(distance - last_distance) 
                # Abortを回避する処理
                if delta_distance < 0.1: # 進んだ距離が0.1[m]よりすくない
                    stay_time = rospy.Time.now() - begin_navigation
                    if stay_time.to_sec() > 60.0: # 止まっている時間が60秒を経過したら
                        rospy.loginfo("Next!!")   # 次のウェイポイントへ
                        break
                    else : # 30秒おきに現在位置とウェイポイント間の距離を報告
                        verbose_time = rospy.Time.now() - verbose_start
                        if verbose_time.to_sec() > 30.0:
                            rospy.loginfo("Distance to goal: %lf\n", distance)
                            verbose_start = rospy.Time.now()
                else :
                    last_distance = distance
                    begin_navigation = rospy.Time.now()
                
                # distanceが0.5[m]以下で次のウェイポイントへ
                # ウェイポイントが目標値の場合は適用されない
                if distance <= 0.4:
                    break
            i += 1
                
    # Ctrl + C を押して終了するときの処理
    def shutdown(self):
        rospy.loginfo('The robot was terminated')
        # serverへゴールのキャンセルを送る
        self.ac.cancel_goal()

if __name__ == '__main__':
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node('waypoint_navi')
    w = WpNavi()
    try:
        w.way_point()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Wapoint navigation finished")
        
