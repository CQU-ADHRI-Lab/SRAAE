#!/usr/bin/python3.8
from urllib import response
import rospy
from rospy.timer import Rate, sleep
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

import numpy as np
import time
import os

map_area = {"small_house":73280,"my_house":172200, "office":447800, "small":111800, "large":234500, "non":145400}


class ExpNode:
    def __init__(self, robot_list) -> None:
        self.map = None
        self.start_time = None
        self.end_time = None
        self.exp_time = None
        self.exploration = 0
        self.new_data = MarkerArray()
        self.trajectory_data_list = [[] for _ in robot_list]  # 保存每个机器人的轨迹数据列表
        rospy.Subscriber("/start_exp", String, self.exp_start_callback)
        self.trajectory_correcter_pub = rospy.Publisher("/trajectory_correcter", MarkerArray, queue_size=1)
        self.exp_start = 0
        self.exp_done = 0
        print("before callback")
        print("EXP START!")
        self.start_time = time.time()
        rospy.Subscriber(
            "/robot1/map", OccupancyGrid, self.map_callback, queue_size=1)
        self.trajectory_point = [None for robot in robot_list]
        self.trajectory_length = np.zeros(len(robot_list))
        self.half_length = None
        for robot in robot_list:
            rospy.Subscriber(
                robot+"/trajectory_node_list", MarkerArray, self.trajectory_length_callback, callback_args=robot, queue_size=1)
        self.file_path = "/home/oseasy/YOEO_ws/exploration_rate/tare_large_office1.txt"

    def map_callback(self, data):
        shape = (data.info.height, data.info.width)
        self.map = np.asarray(data.data).reshape(shape)
        map_size = np.count_nonzero(self.map>=0)
        map_size -= np.count_nonzero(self.map==1)
        explore_rate = map_size / map_area["office"]
        self.exploration = explore_rate
        self.exp_time_real = time.time() -self.start_time
 
        if explore_rate>0.95:
            print("almost done !!!!!!!!!!")
        print(self.exp_time_real,self.trajectory_length[0],explore_rate)
        if not os.path.exists(self.file_path):
            open(self.file_path, 'w').close()
        with open(self.file_path, 'a') as f:
            f.write(f"{self.trajectory_length[0]} {explore_rate}\n")
    
    def trajectory_length_callback(self, data, robot):
        num = int(robot[5:]) - 1
        if self.trajectory_point[num] == None:
            self.trajectory_point[num] = data.markers[2].points[-1]
        # 修改轨迹点的颜色为绿色
        data.markers[2].color.r = 0.0
        data.markers[2].color.g = 1.0
        data.markers[2].color.b = 0.0
        # 修改轨迹的粗细
        data.markers[2].scale.x = 0.2
        data.markers[2].scale.y = 0.2
        data.markers[2].scale.z = 0.2
        temp_position = data.markers[2].points[-1]
        point1 = np.asarray([self.trajectory_point[num].x, self.trajectory_point[num].y])
        point2 = np.asarray([temp_position.x, temp_position.y])
        self.trajectory_length[num] += np.linalg.norm(point1 - point2)
        self.trajectory_point[num] = temp_position


    def exp_start_callback(self, data):
        self.exp_start = 1
        if self.exp_done == 1:
            self.exp_time = time.time()-self.start_time
            print("exp done,", self.exp_time, self.half_length)
        

if __name__ == '__main__':
    rospy.init_node('experiment_node')
    robot_num = rospy.get_param("~robot_num")
    print("Experiment start:", robot_num)
    robot_list = list()
    for rr in range(robot_num):
        robot_list.append("robot"+str(rr+1))
    node = ExpNode(robot_list)
    rospy.spin()
