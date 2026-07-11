#!/usr/bin/env python3.8
import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64
import time

class GoalTimePlotter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('goal_time_plotter', anonymous=True)
        
        # 存储数据的列表
        self.goal_sequence = []  # goal序号
        self.goal_times = []     # 到达目标的时间（秒）
        
        # 创建绘图窗口
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        # 设置坐标轴标签并放大字号
        self.ax.set_xlabel('Number of Goals', fontsize=16)
        self.ax.set_ylabel('Goal Reach Time (s)', fontsize=16)
        self.ax.grid(True)
        
        # 订阅目标时间话题
        rospy.Subscriber('/robot1/goal_reach_time', Float64, self.goal_time_callback)
        
        # 设置动态绘图
        plt.ion()
        plt.show()
        
        # 注册程序关闭时的回调函数
        rospy.on_shutdown(self.print_statistics)

    def goal_time_callback(self, msg):
        # 计算goal序号（从1开始）
        goal_number = len(self.goal_times) + 1
        
        # 添加数据到列表
        self.goal_sequence.append(goal_number)
        self.goal_times.append(msg.data)
        
        # 更新绘图
        self.update_plot()

    def update_plot(self):
        # 清除当前图形
        self.ax.clear()
        
        # 绘制数据
        self.ax.plot(self.goal_sequence, self.goal_times, 'b-', linewidth=2, marker='o', markersize=5)
        
        # 设置图表属性
        self.ax.set_xlabel('Number of Goals', fontsize=16)
        self.ax.set_ylabel('Goal Reach Time (s)', fontsize=16)
        
        # 设置x轴从1开始，自动调整y轴范围
        if self.goal_sequence:
            self.ax.set_xlim(0.5, max(self.goal_sequence) + 0.5)
            y_min = max(0, min(self.goal_times) - 0.5)
            y_max = max(self.goal_times) + 0.5
            self.ax.set_ylim(y_min, y_max)
        
        # 添加网格线
        self.ax.grid(True, linestyle='--', alpha=0.7)
        
        # 刷新图形
        self.fig.canvas.flush_events()

    def print_statistics(self):
        """打印目标到达时间的统计信息"""
        if not self.goal_times:
            rospy.loginfo("No goal times recorded.")
            return
            
        # 计算总目标数
        total_goals = len(self.goal_times)
        
        # 计算超过2.2秒的目标数
        over_2_2 = sum(1 for t in self.goal_times if t > 2.2)
        over_2_2_percent = (over_2_2 / total_goals) * 100
        
        # 计算超过3.1秒的目标数
        over_3_1 = sum(1 for t in self.goal_times if t > 3.1)
        over_3_1_percent = (over_3_1 / total_goals) * 100
        
        # 计算平均时间
        avg_time = sum(self.goal_times) / total_goals
        
        # 打印统计结果
        rospy.loginfo("\n===== Goal Time Statistics =====")
        rospy.loginfo(f"Total goals: {total_goals}")
        rospy.loginfo(f"Goals over 2.2 seconds: {over_2_2} ({over_2_2_percent:.2f}%)")
        rospy.loginfo(f"Goals over 3.1 seconds: {over_3_1} ({over_3_1_percent:.2f}%)")
        rospy.loginfo(f"Average goal time: {avg_time:.2f} seconds")

    def run(self):
        # 主循环
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            plt.pause(0.1)  # 暂停以更新图形
            rate.sleep()

if __name__ == '__main__':
    try:
        plotter = GoalTimePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        plt.close('all')
        pass

