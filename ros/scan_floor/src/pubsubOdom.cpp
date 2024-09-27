#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <zmq.hpp>
#include <Eigen/Dense>

class Odometry
{
public:
    Odometry() : nh_(), context_(1), socket_(context_, zmq::socket_type::pull)
    {
        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1);

        try {
            socket_.bind("tcp://*:5555");
        } catch (const zmq::error_t& e) {
            ROS_ERROR_STREAM("Failed to bind ZeroMQ socket: " << e.what());
            ros::shutdown();
            return;
        }

        timer_ = nh_.createTimer(ros::Duration(0.001), &Odometry::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher pub_odom_;
    zmq::context_t context_;
    zmq::socket_t socket_;
    nav_msgs::Odometry odom_;

    void timerCallback(const ros::TimerEvent&)
    {
        zmq::message_t message;
        try {
            zmq::recv_result_t result = socket_.recv(message, zmq::recv_flags::dontwait);
            if (!result.has_value()) {
                return;  // No message available, don't block
            }
        } catch (const zmq::error_t& e) {
            ROS_ERROR_STREAM("Failed to receive ZeroMQ message: " << e.what());
            return;
        }

        if (message.size() != sizeof(double) * 16) {
            ROS_ERROR("Received message size incorrect");
            return;
        }

        double* matrix = static_cast<double*>(message.data());
        Eigen::Matrix4d cam_pose_cw;
        cam_pose_cw << matrix[ 0], matrix[ 1], matrix[ 2], matrix[ 3],
                       matrix[ 4], matrix[ 5], matrix[ 6], matrix[ 7],
                       matrix[ 8], matrix[ 9], matrix[10], matrix[11],
                       matrix[12], matrix[13], matrix[14], matrix[15];

        Eigen::Matrix3d rotation_matrix = cam_pose_cw.block<3, 3>(0, 0);
        std::cout << rotation_matrix << std::endl;
        std::cout << "=================" << std::endl;
        Eigen::Matrix3d rot_inv = rotation_matrix.inverse();
        Eigen::Vector3d euler_angles = rot_inv.eulerAngles(2, 1, 0);

        odom_.header.stamp = ros::Time::now();
        odom_.header.frame_id = "base_link";
        odom_.pose.pose.position.x =  matrix[14];
        odom_.pose.pose.position.y = -matrix[12];
        odom_.pose.pose.position.z = -matrix[13];
        
        // TODO: Set orientation using euler angles

        pub_odom_.publish(odom_);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pubsub_odom");
    Odometry odometry;
    ros::spin();
    return 0;
}