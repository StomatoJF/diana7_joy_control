#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import DianaApi, time
import rospy
from diana7_msgs.msg import CartesianState,Joint
from std_msgs.msg import Bool,Float64MultiArray

class Diana7ArmControl:
    def __init__(self):
        rospy.init_node("Diana7_Arm_Control")
        self.ipAddress = "192.168.10.75"
        self.acc = 0.4
        self.vel_division = 5  # 速度除数，除以手柄输入的速度，让速度变小一点
        self.free_driving_flag = False
        self.JOINT_NUM = 6
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #0-6号关节的速度rad/s
        DianaApi.initSrv((self.ipAddress, 0, 0, 0, 0, 0))
        timer = rospy.Timer(rospy.Duration(0.01), self.control_loop_timer_callback)
        rospy.Subscriber(
            "joint_control_topic", Joint, self.control_callback
        )  # 读取机器人当前控制速度和角速度
        rospy.Subscriber(
            "free_driving_flag", Bool, self.free_driving_callback
        )  # 自由驱动与否

    def control_loop_timer_callback(self, event):  # 定时执行控制指令
        if not self.free_driving_flag:
            rospy.loginfo(self.joints)
            DianaApi.speedJ(self.joints, self.acc, 0, self.ipAddress)

    def control_callback(self, data):  # 根据收到的消息控制机械臂运动
        self.joints[0] = data.J[0]/self.vel_division
        self.joints[1] = data.J[1]/self.vel_division
        self.joints[2] = data.J[2]/self.vel_division
        self.joints[3] = data.J[3]/self.vel_division
        self.joints[4] = data.J[4]/self.vel_division
        self.joints[5] = data.J[5]/self.vel_division
        self.joints[6] = data.J[6]/self.vel_division
        




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
