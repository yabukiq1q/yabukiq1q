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

///////////////////////////////////////////////////
//ポート名の定義
std::string serialPortName;

//ボーレートの定義
int baudrate = 115200;

//ファイルディスクリプタ
int file_descriptor;

//シリアル通信系
void openPort(const char *device_name, int baudrate_int);
void closePort();
bool tryConvertingBaudrate(unsigned int &BAUDRATE, int baudrate_int);

//シリアル通信でデータを受け取る間隔
double readInterval = 0.02;
unsigned char data[10]; //data header 1 byte data 4byte data 4byte footer 1 byte
///////////////////////////////////////////////////

class SerialCommunication
{
    public:
        SerialCommunication() : nh(), tfBuffer(), tfListener(tfBuffer)
        {
            getParameters();
            openPort(serialPortName.c_str(), baudrate);

            pub_receivedvalues = nh.advertise<std_msgs::String>("serial_data", 1);
            pub_base_link      = nh.advertise<geometry_msgs::Pose>("base_link", 1);
            sub_cmd_vel        = nh.subscribe("cmd_vel", 1, &SerialCommunication::callback_cmd_vel, this); 

            timer = nh.createTimer(ros::Duration(readInterval), [&](const ros::TimerEvent& e)
            {
                write2dynamixel();
            });

            timer2 = nh.createTimer(ros::Duration(0.001), [&](const ros::TimerEvent& e)
            {
                readFromDynamixel();
            });
        }

        ~SerialCommunication()
        {
            float a = 0.0;
            data[0] = 0xFF;
            data[9] = 0xFE;
            memcpy(&data[1], &a, sizeof(a));
            memcpy(&data[5], &a, sizeof(a));
            int write_data_size = write(file_descriptor, data , sizeof(data));

            ROS_INFO("Port close");
            closePort();
        }

    private:
        ros::NodeHandle nh;
        ros::Timer timer;
        ros::Timer timer2;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        ros::Publisher  pub_receivedvalues;
        ros::Publisher  pub_base_link;
        ros::Subscriber sub_cmd_vel;
        std::string receive_msg;
        void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg);
        void getParameters();
        void write2dynamixel();
        void readFromDynamixel();
        geometry_msgs::Pose zeroPoint;
        geometry_msgs::Pose currentPose;
        void publisher_base_link();
};

void SerialCommunication::callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    float tmp1 = (float)msg->linear.x;
    float tmp2 = (float)msg->angular.z;
    //ROS_INFO("x : %+.3f, z : %+.3f", tmp1, tmp2);
    memcpy(&data[1], &tmp1, sizeof(float));
    memcpy(&data[5], &tmp2, sizeof(float));
}

void SerialCommunication::getParameters()
{
    nh.param("/serialCommunication/serialPortName", serialPortName, std::string("/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00"));
    nh.param("/serialCommunication/baudrate", baudrate, 115200);
    nh.param("/serialCommunication/readInterval", readInterval, 0.02);
}

void SerialCommunication::write2dynamixel()
{
    data[0] = 0xFF;
    data[9] = 0xFE;
    int write_data_size = write(file_descriptor, data , sizeof(data));
    //ROS_INFO("write data size: %d", write_data_size);
}

void SerialCommunication::readFromDynamixel()
{
    unsigned char read_buf[0xFF];
    int read_data_size = read(file_descriptor, &read_buf, sizeof(read_buf));

    if (read_data_size <= 0)
    {
        //ROS_WARN("Failed to read from the serial port");
        return;
    }
    else
    {
        ROS_INFO("read_data_size: %d", read_data_size);
    }
    
    int index = 0;
    while(read_buf[index] != 'S' && index < read_data_size)
    {
        index++;
    }

    //25 = doubleのデータ×3 + フッター  
    if (index + 25 < read_data_size && read_buf[index + 25] == 'P')
    {
        std::memcpy(&currentPose.position.x, &read_buf[index + 1], sizeof(double));
        std::memcpy(&currentPose.position.y, &read_buf[index + 1 + sizeof(double)], sizeof(double));
        double yaw;
        std::memcpy(&yaw, &read_buf[index + 1 + 2 * sizeof(double)], sizeof(double));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        currentPose.orientation.w = q.w();
        currentPose.orientation.x = q.x();
        currentPose.orientation.y = q.y();
        currentPose.orientation.z = q.z();

        static bool once = true;
        if(once)
        {
            zeroPoint = currentPose;
            once = false;
        }
    }
    else
    {
        //ROS_INFO("READ ERROR: Invalid data structure");
    }
}

void SerialCommunication::publisher_base_link()
{
    geometry_msgs::Pose p;
    p.position.x = currentPose.position.x - zeroPoint.position.x;
    p.position.y = currentPose.position.y - zeroPoint.position.y;
    p.position.z = currentPose.position.z - zeroPoint.position.z;
    tf2::Quaternion q1, q2;
    tf2::fromMsg(zeroPoint.orientation,   q1);
    tf2::fromMsg(currentPose.orientation, q2);
    tf2::Quaternion q_result = q1 * q2;
    p.orientation = tf2::toMsg(q_result);
    pub_base_link.publish(p);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_communication");
    SerialCommunication serialCommunication;
    ros::spin();
}

void openPort(const char *device_name, int baudrate_int)
{
    file_descriptor = open(device_name, O_RDWR);
    device_name = (char*)device_name;

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(file_descriptor, &tty) != 0)
    {
        ROS_ERROR("Serial Failed 1: %s\n", device_name);
        return;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 0;

    // Set in/out baud rate
    unsigned int BAUDRATE;
    if (tryConvertingBaudrate(BAUDRATE, baudrate_int) == false)
    {
        printf("Invalid BAUDRATE : %s\n", device_name);
        return;
    }
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // Save tty settings, also checking for error
    if (tcsetattr(file_descriptor, TCSANOW, &tty) != 0)
    {
        printf("Serial Failed 2: %s\n", device_name);
        return;
    }

    if (file_descriptor < 0)
    {
        printf("Serial Failed 0: %s\n", device_name);
    }
    else
    {
        printf("Serial Open : %s\n", device_name);
    }
}

void closePort()
{
    close(file_descriptor);
    printf("Serial port closed.\n");
}

bool tryConvertingBaudrate(unsigned int &BAUDRATE, int baudrate_int)
{
    switch(baudrate_int)
    {
        case 0       : BAUDRATE = B0;       break;
        case 50      : BAUDRATE = B50;      break;
        case 75      : BAUDRATE = B75;      break;
        case 110     : BAUDRATE = B110;     break;
        case 134     : BAUDRATE = B134;     break;
        case 150     : BAUDRATE = B150;     break;
        case 200     : BAUDRATE = B200;     break;
        case 300     : BAUDRATE = B300;     break;
        case 600     : BAUDRATE = B600;     break;
        case 1200    : BAUDRATE = B1200;    break;
        case 1800    : BAUDRATE = B1800;    break;
        case 2400    : BAUDRATE = B2400;    break;
        case 4800    : BAUDRATE = B4800;    break;
        case 9600    : BAUDRATE = B9600;    break;
        case 19200   : BAUDRATE = B19200;   break;
        case 38400   : BAUDRATE = B38400;   break;
        case 57600   : BAUDRATE = B57600;   break;
        case 115200  : BAUDRATE = B115200;  break;
        case 230400  : BAUDRATE = B230400;  break;
        case 460800  : BAUDRATE = B460800;  break;
        case 500000  : BAUDRATE = B500000;  break;
        case 576000  : BAUDRATE = B576000;  break;
        case 921600  : BAUDRATE = B921600;  break;
        case 1000000 : BAUDRATE = B1000000; break;
        case 1152000 : BAUDRATE = B1152000; break;
        case 1500000 : BAUDRATE = B1500000; break;
        case 2000000 : BAUDRATE = B2000000; break;
        case 2500000 : BAUDRATE = B2500000; break;
        case 3000000 : BAUDRATE = B3000000; break;
        case 3500000 : BAUDRATE = B3500000; break;
        case 4000000 : BAUDRATE = B4000000; break;
        default      : return false;
    }
    return true;
}
