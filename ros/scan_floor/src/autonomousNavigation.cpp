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
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <math.h>
#include <cstdio>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

using namespace std;

class AutoMove
{
    public:
        AutoMove() : nh(), tfBuffer(), tfListener(tfBuffer)
        {
            getParameters();
            readCSV(pathName);

            pub_cmd_vel   = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            //sub_odometry  = nh.subscribe("odometry", 1, &AutoMove::callback, this);
            sub_targetPos = nh.subscribe("odometry", 1, &AutoMove::callback_odom, this);

            timer = nh.createTimer(ros::Duration(loopRate), [&](const ros::TimerEvent& e)
            {
                sendCmd_vel();
            });
        }

        ~AutoMove()
        {
        }

    private:
        ros::NodeHandle nh;
        ros::Timer timer;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        double loopRate = 0.1;
        ros::Publisher  pub_cmd_vel;
        ros::Subscriber sub_odometry;
        ros::Subscriber sub_targetPos;
        geometry_msgs::Twist cmd_vel;
        std::vector<std::vector<double>> checkpoints; //通過点の座標
        std::string pathName;
        geometry_msgs::Pose currentGoal;
        void callback(const nav_msgs::Odometry::ConstPtr& msg);
        void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
        void sendCmd_vel();
        void readCSV(std::string path);
        void getParameters();
        void jokou();
};

void AutoMove::jokou()
{
    cmd_vel.linear.x  = 40.0;
    cmd_vel.angular.z =  0.0;
}

void AutoMove::callback(const nav_msgs::Odometry::ConstPtr& msg)
{

}

void AutoMove::callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    cmd_vel.linear.x  = 50.0 * (currentGoal.position.x - msg->pose.pose.position.x);
    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, -30.0), 30.0);
    //cmd_vel.angular.z = currentGoal.orientation.z - msg->pose.pose.z;
    cmd_vel.angular.z = 0.0;
}

void AutoMove::readCSV(std::string path)
{
    std::ifstream csvFile(path);
    if (!csvFile.is_open())
    {
        ROS_ERROR("Couldn't open CSV File : %s", path.c_str());
        return;
    }
    std::string line;
    while (std::getline(csvFile, line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> rowData;
        // カンマで区切られた各セルを取得し、doubleに変換してベクターに追加
        while (std::getline(lineStream, cell, ','))
        {
            rowData.push_back(std::stod(cell));
        }
        //z軸の値がなければ0を挿入
        if(rowData.size() < 3)
        {
            rowData.push_back(0.0);
        }
        checkpoints.push_back(rowData);
    }

    ROS_INFO("AAA");
    for(int i = 0; i < checkpoints.size(); i++)
    {
        for(int j = 0; j < checkpoints[i].size(); j++)
        {
            std::cout << checkpoints[i][j] << ", ";
        }
        std::cout << std::endl;
    }
    ROS_INFO("BBB");

    currentGoal.position.x = checkpoints[1][0];
    std::cout << currentGoal.position.x << std::endl;
}

void AutoMove::sendCmd_vel()
{  
    std::cout << cmd_vel.linear.x << std::endl;
    pub_cmd_vel.publish(cmd_vel);
}

void AutoMove::getParameters()
{
    std::cout << nh.param("/autonomousNavigation/pathName", pathName, std::string("/home/ubuntu/scanfloor_ws/src/scan_floor/config/target.csv")) << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_automatic");
    AutoMove AutoMove;
    ros::spin();
}
