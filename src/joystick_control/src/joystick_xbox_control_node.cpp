#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include "diana7_msgs/CartesianState.h"
class JoystickPublisher
{
public:
    JoystickPublisher()
    {
        // 创建机械臂控制消息发布者
        pub_ = nh_.advertise<diana7_msgs::CartesianState>("control_topic", 10);
        free_pub_ = nh_.advertise<std_msgs::Bool>("free_driving_flag", 10);
        // 订阅原始手柄消息节点
        sub_ = nh_.subscribe("joy", 10, &JoystickPublisher::joyCallback, this);
    }
    float low_pass_filter(float data)
    {
        if (abs(data) > 0.1)
        {
            return data;
        }
        else
        {
            return 0;
        }
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
    {
        // 获取摇杆的值
        float left_stick_x = low_pass_filter(joy->axes[0]);
        float left_stick_y = low_pass_filter(joy->axes[1]);
        float right_stick_x = low_pass_filter(joy->axes[2]);
        float right_stick_y = low_pass_filter(joy->axes[3]);
        float left_trigger = low_pass_filter(joy->axes[4]);
        float right_trigger = low_pass_filter(joy->axes[5]);

        // 获取十字按键的状态
        float button_left_right = joy->axes[6];
        float button_up_down = joy->axes[7];

        // 获取按键的状态
        bool button_a = joy->buttons[0];
        bool button_b = joy->buttons[1];
        bool button_x = joy->buttons[2];
        bool button_y = joy->buttons[3];

        bool button_free = joy->buttons[12];

        // ROS_INFO("lx=(%0.2f) ly=(%0.2f) rx=(%0.2f) ry=(%0.2f) lt=(%0.2f) rt=(%0.2f)", left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger);
        // ROS_INFO("ba=(%d) bb=(%d) bx=(%d) by=(%d)", button_a, button_b, button_x, button_y);
        // ROS_INFO("blr=(%f) bud=(%f)", button_left_right, button_up_down);
        // ROS_INFO("bf=(%f)", button_free );
    

        diana7_msgs::CartesianState CartesianState_msg;
        CartesianState_msg.twist.linear.x = -right_stick_y;
        CartesianState_msg.twist.linear.y = left_stick_y;
        CartesianState_msg.twist.linear.z = -left_stick_x;
        CartesianState_msg.twist.angular.x = -right_stick_x;
        CartesianState_msg.twist.angular.y = button_left_right;
        CartesianState_msg.twist.angular.z = button_up_down;

        pub_.publish(CartesianState_msg);
        // 只有当6号按键按下使按钮状态改变的时候才会发布这个话题，其他按键按下的时候不会发布，避免重复触发
        if (free_driving_old_status != button_free)
        {
            std_msgs::Bool free_flag_msg;
            free_flag_msg.data = button_free;
            free_driving_old_status = button_free;
            free_pub_.publish(free_flag_msg);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher free_pub_;
    ros::Subscriber sub_;
    bool free_driving_old_status = false;
    // // 创建低通滤波器对象，设置阻尼系数
    // LowPassFilter filter(0.5);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_publisher_node");
    JoystickPublisher joystickPublisher;

    ros::spin();

    return 0;
}
