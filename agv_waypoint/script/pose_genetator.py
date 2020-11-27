#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
from math import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class SavePose():
    def __init__(self):
        rospy.on_shutdown(self.shutdown) # Ctrl+cが押されるとself.shutdownが呼び出される
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback)
        
        self.currnt_path = os.getcwd() # 現在のディレクトリ取得
        self.csv_name = "pose.csv" # csvファイル名
        self.file_path = os.path.join(self.currnt_path, self.csv_name) # currnt_path + csv_name

        self.x, self.y = 0.0, 0.0 # ロボットの位置を初期化
        self.qz, self.qw = 0.0, 0.0 # ロボットの向きを初期化
        self.last_x = self.x # 前のロボットのx座標を記録
        self.last_y = self.y # 前のロボットのy座標を記録

    def callback(self, pose_msg):
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
        if length > 0.5:
            rospy.loginfo("Successed get new data")
            position_data = [self.x, self.y, 0, 0, 0, self.qz, self.qw]
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
        #finish_data = [999,999,999,999,999,999,999]
        #with open(self.csv_name, "a") as f:
            #writer = csv.writer(f)
            #writer.writerow(finish_data)
        rospy.loginfo("Pose save finish")
            

if __name__ == "__main__":
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    rospy.init_node("pose_save")
    try :       
        SavePose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
       