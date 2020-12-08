#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
from math import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String

class SaveWp():
    def __init__(self):
        rospy.on_shutdown(self.shutdown) # Ctrl+cが押されるとself.shutdownが呼び出される

        self.pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.goal_sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)
        self.result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_callback)

        self.currnt_path = os.getcwd() # 現在のディレクトリ取得
        self.csv_name = rospy.get_param("~csv_name","1203.csv") # csvファイル名
        self.file_path = os.path.join(self.currnt_path, self.csv_name) # currnt_path + csv_name

        self.x, self.y, self.goal_x, self.goal_y = 0.0, 0.0, 0.0, 0.0 # ロボットの位置を初期化
        self.qz, self.qw, self.goal_qz, self.goal_qw = 0.0, 0.0, 0.0, 0.0 # ロボットの向きを初期化
        self.last_x = self.x # 前のロボットのx座標を記録
        self.last_y = self.y # 前のロボットのy座標を記録
        self.count = 1
    
    def result_callback(self, result_msg):
        self.status = result_msg.status.status
        goal_data = [self.count, 1, self.goal_x, self.goal_y, 0, 0,
                     0, self.goal_qz, self.goal_qw]
        if self.status == 3:
            with open(self.csv_name, "a") as f:
                writer = csv.writer(f)
                writer.writerow(goal_data)
            rospy.loginfo("Successed get new goal_data")
            self.count += 1

    def goal_callback(self, goal_msg):
        self.goal_x = goal_msg.goal.target_pose.pose.position.x
        self.goal_y = goal_msg.goal.target_pose.pose.position.y
        self.goal_qz = goal_msg.goal.target_pose.pose.orientation.z
        self.goal_qw = goal_msg.goal.target_pose.pose.orientation.w

    def pose_callback(self, pose_msg):
        # file_path名のファイルがあればTreu,なければFalseを返す
        file_check = os.path.isfile(self.file_path)
        self.create_csv(file_check)

        # ロボットの現在の位置と向きを取得
        data = pose_msg.pose.pose
        # ロボットの位置
        self.x = data.position.x
        self.y = data.position.y
        # ロボットの向き 
        self.qz = data.orientation.z
        self.qw = data.orientation.w
        # ロボットが移動した距離
        length = sqrt(pow((self.x-self.last_x), 2) + pow((self.y-self.last_y), 2))

        # lenghtが0.5[m]より大きければposition_dataをcsvファイルに書き込む
        if length > 0.4:
            rospy.loginfo("Successed get new pose_data")
            position_data = [0, 0, self.x, self.y, 0, 0, 0, self.qz, self.qw]
            with open(self.csv_name, "a") as f:
                writer = csv.writer(f)
                writer.writerow(position_data)
            # lenght>0.5の時だけロボットの位置を記録する
            self.last_x = self.x
            self.last_y = self.y

    # 既存のファイルがなければ新たなcsvファイル作成
    def create_csv(self, file_check):
        # Falseで新しくcsvファイルを作成
        if not file_check:
            print("Create new csv file")
            with open(self.csv_name, "w") as f:
                writer = csv.writer(f)
        # Trueだとなにもしない
        else :
            pass

    # Ctrl+cが押されたら時の処理
    def shutdown(self):
        rospy.loginfo("Pose save finish")
            
if __name__ == "__main__":
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node("waypoint_generator")
    try :       
        SaveWp()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass