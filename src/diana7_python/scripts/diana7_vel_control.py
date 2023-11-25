#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import DianaApi, time
import rospy
from diana7_msgs.msg import CartesianState
from std_msgs.msg import Bool


class Diana7ArmControl:
    def __init__(self):
        rospy.init_node("Diana7_Arm_Control")
        self.ipAddress = "192.168.10.75"
        self.speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 前面三个是笛卡尔空间的线速度，后面三个是角速度
        self.acc = (0.5, 0.5)
        self.vel_division=5#速度除数，除以手柄输入的速度，让速度变小一点
        self.free_driving_flag = False
        DianaApi.initSrv((self.ipAddress, 0, 0, 0, 0, 0))
        timer = rospy.Timer(rospy.Duration(0.01), self.control_loop_timer_callback)
        rospy.Subscriber(
            "control_topic", CartesianState, self.control_callback
        )  # 读取机器人当前控制速度和角速度
        rospy.Subscriber(
            "free_driving_flag", Bool, self.free_driving_callback
        )  # 自由驱动与否
    
    def control_loop_timer_callback(self, event):  # 定时执行控制指令
        if not self.free_driving_flag:
            # rospy.loginfo(self.speeds)
            DianaApi.speedL(self.speeds, self.acc, 0, self.ipAddress)

    def control_callback(self, cartesian_state):  # 根据收到的消息控制机械臂运动
        self.speeds[0] = cartesian_state.twist.linear.x / self.vel_division
        self.speeds[1] = cartesian_state.twist.linear.y / self.vel_division
        self.speeds[2] = cartesian_state.twist.linear.z / self.vel_division
        self.speeds[3] = cartesian_state.twist.angular.x / self.vel_division
        self.speeds[4] = cartesian_state.twist.angular.y / self.vel_division
        self.speeds[5] = cartesian_state.twist.angular.z / self.vel_division
    def free_driving_callback(self, free_driving_flag):
        self.free_driving_flag = free_driving_flag.data
        rospy.loginfo(self.free_driving_flag)
        time.sleep(0.5)
        if self.free_driving_flag:
            DianaApi.freeDriving(
                DianaApi.freedriving_mode_e.E_NORMAL_FREEDRIVING, self.ipAddress
            )
        else:
            DianaApi.freeDriving(
                DianaApi.freedriving_mode_e.E_DISABLE_FREEDRIVING, self.ipAddress
            )


if __name__ == "__main__":
    try:
        diana7_arm_control = Diana7ArmControl()
        rospy.spin()
        # goal_publisher.move_to_goals()
    except rospy.ROSInterruptException:
        DianaApi.destroySrv()
        pass
