#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import csv, os
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal

def callback(msg):
    # numでウェイポイント数を取得
    with open(csv_name, "r") as f:
        reader = csv.reader(f)
        columns = next(reader)
        row = [row for row in reader]
        num = len(row) + 1

    # robotの位置を取得
    pos = msg.goal.target_pose.pose
    # csvファイルに格納するデータ
    data = [num,pos.position.x,pos.position.y, 0, 0,
            0, pos.orientation.z, pos.orientation.w]

    # numpy.ndarray型に変換
    row_numpy = [row_numpy[1:] for row_numpy in row]
    row_numpy = np.array(row_numpy, dtype=float)
    data_numpy = np.array(data[1:])

    # row_numpyとdata_numpyを比較して,一致するlistがあればTrue,なければFalseを返す
    data_check = (row_numpy == data_numpy).all(axis=1).any()

    # 新しいデータであればcsvファイルに書き込む,既存のデータであればpass
    if not data_check: # False
        rospy.loginfo("Successed get new data")
        write_csv(data)
    else : # True
        rospy.loginfo("This data already exists")
        pass

def write_csv(data):
    # csvファイルにデータ書き込み
    with open(csv_name, "a") as f:
        writer = csv.writer(f)
        writer.writerow(data)

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
    rospy.init_node('waypoint_creat')
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, callback)
    rospy.spin()     
