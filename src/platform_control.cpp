#include <iostream>

// ros 相關
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// 自行加入的功能
#include "kinova_test/HumanState.h"
#include "kinova_test/conio.h"
#include "kinova_test/controller.h"

int main(int argc, char **argv)
{
    // ROS
    ros::init(argc, argv, "platformDevice"); // rosnode的名稱
    HumanState humanState;
    // ROS
    ros::NodeHandle n;
    ros::Publisher msg_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000); // rostopic的名稱(Publish)
    ros::Subscriber sub = n.subscribe("xsens2kinova", 1000, &HumanState::updateHumanData, &humanState);

    geometry_msgs::Twist twist;
    while (ros::ok())
    {
        if (!_kbhit())
        {
            platform_control(humanState, twist);
        }
        else
        {
            if ((char)_getch() == 's')
            {
                twist.linear.x = 0;
                twist.angular.z = 0;
            }
            msg_pub.publish(twist);
            break;
        }
        msg_pub.publish(twist);
        ros::spinOnce();
    }
    return 0;
}
