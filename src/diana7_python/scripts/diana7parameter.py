#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import DianaApi,time
from std_msgs.msg import String 
from diana7_msgs.msg import Joint
"""
    获取机械臂关节参数：
                    1.导包；
                    2.初始化ros节点；
                    3.创建发布者对象；
                    4.编写发布逻辑并发布数据；
                    5.spin()

"""

if __name__ == "__main__":
    # 2.初始化ros节点；
    rospy.init_node("Diana7parameter")
    ipAddress = "192.168.10.75"
    joints =[0.0]*7
    DianaApi.initSrv((ipAddress, 0, 0, 0, 0, 0))
    # 3.创建发布者对象；
    pub = rospy.Publisher("joint_parameter",Joint,queue_size=10)
    # 4.编写发布逻辑并发布数据；
    # 获取数据
    DianaApi.getJointPos(joints, ipAddress)
    # 创建数据
    pos_msg = Joint()
    #使用循环发布数据
    while not rospy.is_shutdown():
        pos_msg.J[0]=joints[0]
        pos_msg.J[1]=joints[1]
        pos_msg.J[2]=joints[2]
        pos_msg.J[3]=joints[3]
        pos_msg.J[4]=joints[4]
        pos_msg.J[5]=joints[5]
        pos_msg.J[6]=joints[6]
        pub.publish(pos_msg)
        
    #发布数据
    
    
    