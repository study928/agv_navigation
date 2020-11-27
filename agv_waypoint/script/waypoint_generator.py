#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
from move_base_msgs.msg import MoveBaseActionGoal

def callback(msg):
    # numでウェイポイント数を取得
    with open(csv_name, "r") as f:
        reader = csv.reader(f)
        num = len([row for row in reader])

    # robotの位置を取得
    pos = msg.goal.target_pose.pose
    # csvファイルに格納するデータ
    data = [num,pos.position.x,pos.position.y, 0, 0,
            0, pos.orientation.z, pos.orientation.w]
    print('{0},{1},0.0,0.0,0.0,{2},{3}'.format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))

    # csvファイルにデータ書き込み
    with open(csv_name, "a") as f:
        writer = csv.writer(f)
        writer.writerow(data)
    rospy.loginfo('Successed get data')

# 新しいcsvファイルを作成する
def create_csv(csv_name, mycheck):
    culumns = ["num","x","y","z","q1","q2","q3","q4"]
    # Falseだったら新規ファイル作成
    if not mycheck: 
        with open(csv_name, "w") as f:
            writer = csv.writer(f)
            writer.writerow(culumns) 
    # Trueだったらpass 
    else : 
        pass

# 既存のファイルチェック
def main():
    # mycheckはファイルがあったらTrue,なかったらFalseを返す
    mycheck = os.path.isfile(file_path)
    
    if not mycheck:
        print("Create new csv file")
    else :
        print("A csv file with that name already exists")
    create_csv(csv_name, mycheck)

if __name__ == "__main__":
    # csvファイルのあるディレクトリへ移動
    os.chdir("/home/nvidia/catkin_ws/src/agv_navigation/agv_waypoint/waypoint")
    currnt_path = os.getcwd() # 現在のディレクトリ
    csv_name = "1125.csv" # csvファイル名
    file_path = os.path.join(currnt_path, csv_name) # 現在のディレクトリとcsvファイル名を合成
    # main関数呼び出し
    main()

    # nodeの初期化
    rospy.init_node('waypoint_generator')
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, callback)
    rospy.spin()     
