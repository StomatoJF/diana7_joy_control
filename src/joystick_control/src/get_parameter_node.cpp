#include <ros/ros.h>
#include "std_msgs/String.h"
#include <diana7_msgs/Joint.h>
/*
    机械臂参数发布方实现：
                1.包含头文件；
                消息类型？
                2.初始化ROS节点；

                3.创建节点句柄；
                4.创建订阅者对象；
                
*/
void posCallback(const diana7_msgs::Joint::ConstPtr& msg)
{
    ROS_INFO("joint0=(%0.2f) joint1=(%0.2f) joint2=(%0.2f) joint3=(%0.2f) joint4=(%0.2f) joint5=(%0.2f) joint6=(%0.2f) ",msg->J[0], msg->J[1], msg->J[2], msg->J[3], msg->J[4], msg->J[5],msg->J[6]);
}

int main(int argc, char * argv[])
{
    // 2.初始化ROS节点；
    ros::init(argc, argv,"joint_parameter_node");
    // 3.创建节点句柄；
    ros::NodeHandle nh;
    // 4.创建订阅者对象；
    ros::Subscriber sub = nh.subscribe("joint_parameter",10,posCallback);
    // 5.处理订阅到的数据
    return 0;
}
