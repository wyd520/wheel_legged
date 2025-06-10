#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
import numpy as np
from collections import deque

plt.ion()  # 启用交互模式

class JointsPlotter:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('joints_visualizer', anonymous=True)
        self.start_time = rospy.get_time()
        # 初始化Matplotlib图形和子图
        self.fig, self.axes = plt.subplots(6, 2, figsize=(15, 20))
        plt.subplots_adjust(hspace=0.3, wspace=0.25)
        self.fig.suptitle('Joint Positions and Velocities over Time', fontsize=16, y=0.95)
        
        # 初始化数据存储
        max_points = 500
        self.time_data = deque(maxlen=max_points)
        self.position_data = [deque(maxlen=max_points) for _ in range(12)]  # 存储12个关节的位置
        self.velocity_data = [deque(maxlen=max_points) for _ in range(12)]   # 存储12个关节的速度
        
        # 初始化每个子图的曲线对象
        joint_names = [
            'FL Hip', 'FL Knee', 'FL Wheel',
            'FR Hip', 'FR Knee', 'FR Wheel',
            'RL Hip', 'RL Knee', 'RL Wheel',
            'RR Hip', 'RR Knee', 'RR Wheel'
        ]
        self.lines = []
        for i in range(12):
            row, col = divmod(i, 2)
            ax = self.axes[row, col]
            ax.set_title(joint_names[i], fontsize=10)
            ax.set_xlabel('Time (s)', fontsize=8)
            ax.set_ylabel('Pos (rad) / Vel (rad/s)', fontsize=8)
            ax.tick_params(labelsize=8)
            
            line_pos, = ax.plot([], [], 'r-', lw=1, label='Position')
            line_vel, = ax.plot([], [], 'b--', lw=1, label='Velocity')
            ax.legend(loc='upper right', fontsize=6)
            ax.grid(True, alpha=0.3)
            self.lines.append((line_pos, line_vel))
        
        
        rospy.Subscriber("/jointsPosVel", Float32MultiArray, self.callback)
        
        self.plot_update_interval = 0.1  # 100ms更新
        self.last_plot_time = 0

    def callback(self, msg):
        if len(msg.data) != 24:
            rospy.logwarn_once("Invalid data length: %d (expected 24)", len(msg.data))
            return

        # 记录时间
        current_time = rospy.get_time() - self.start_time
        self.time_data.append(current_time)

        # 解析数据
        for i in range(12):
            self.position_data[i].append(msg.data[i])
            self.velocity_data[i].append(msg.data[i+12])

        # 控制绘图频率
        if current_time - self.last_plot_time >= self.plot_update_interval:
            self.update_plot()
            self.last_plot_time = current_time

    def update_plot(self):
        try:
            times = np.array(self.time_data)
            if len(times) == 0:
                return

            for i in range(12):
                pos = np.array(self.position_data[i])
                vel = np.array(self.velocity_data[i])
                
                line_pos, line_vel = self.lines[i]
                line_pos.set_data(times, pos)
                line_vel.set_data(times, vel)
                
                # 动态调整坐标范围
                ax = self.axes[divmod(i, 2)[0], divmod(i, 2)[1]]
                ax.relim()
                ax.autoscale_view()

            # 优化渲染
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            rospy.logerr("Plot update error: %s", str(e))

if __name__ == '__main__':
    try:
        plotter = JointsPlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass