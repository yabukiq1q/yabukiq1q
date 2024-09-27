#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <math.h>
#include <cstdio>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

using namespace std;

class Joy2cmd
{
    public:
        Joy2cmd() : nh()
        {
            pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            sub_joy = nh.subscribe("/joy", 1, &Joy2cmd::callback, this); 

            timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& e)
            {
                pub_cmd.publish(cmd_vel);
            });
        }

        ~Joy2cmd()
        {
        }

    private:
        ros::NodeHandle nh;
        ros::Timer timer;
        geometry_msgs::Twist cmd_vel;
        ros::Publisher  pub_cmd;
        ros::Subscriber sub_joy;
        void callback(const sensor_msgs::JoyConstPtr& msg);
};

void Joy2cmd::callback(const sensor_msgs::JoyConstPtr& msg)
{
    cmd_vel.linear.x  = 100 * msg->axes[1];
    cmd_vel.angular.z = 100 * msg->axes[4];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy2cmd");
    Joy2cmd Joy2cmd;
    ros::spin();
}
