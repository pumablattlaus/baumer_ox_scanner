#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <std_msgs/Float32.h>

// Scanner:
#include <OXApi/Ox.h>

#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm> // std::find_if
#include <thread>    // std::this_thread::sleep_for
#include <chrono>    // std::chrono::milliseconds
#include <stdio.h>
#include <fstream>
#include <csignal>
#include <sstream>

namespace
{
    volatile std::sig_atomic_t gSignalStatus;
}

class LaserScanPublisher {
public:
    LaserScanPublisher() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Get ROS parameters
        // std::size_t count{0};
        std::string host;
        // int udpPort;
        nh.param<std::string>("host", host, "192.168.0.250");
        // nh.param<int>("udp_port", udpPort, 12345);
        // bool udp; // use UDP protocol
        // nh.param<bool>("udp", udp, false);

        std::string frame_id;
        nh.param<std::string>("frame_id", frame_id, "laser_scanner");
        std::string topic_name;
        nh.param<std::string>("topic_name", topic_name, "point_cloud");

        // Create a publisher object
        // laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
        test_pub = nh.advertise<std_msgs::Float32>("test", 10);

        // Initialize sensor and any other necessary variables
        ox = Baumer::OXApi::Ox::Create( host );
        // opens the network connection
        ox->Connect();
        ROS_INFO_STREAM("Connected to " << host << ".\n");
        ROS_INFO("NOT using UDP.");

        // Main loop to read and publish laser scan data
        ros::Rate loop_rate(400); // 50 Hz, adjust as necessary
        while (ros::ok() && !gSignalStatus) {
            sensor_msgs::PointCloud2 point_msg;
            point_msg.header.frame_id = frame_id;
            // Fill in other necessary fields such as angle_min, angle_max, range_min, range_max, etc.
            // ...

            // Read data from the sensor
            readSensorData(point_msg);
            point_cloud_pub.publish(point_msg);

            // ros::spinOnce();
            loop_rate.sleep();
        }

        // Disconnect from the sensor
        ox->Disconnect();
    }

private:
    ros::Publisher laser_scan_pub;
    ros::Publisher point_cloud_pub;
    // std::shared_ptr<Baumer::OXApi::UdpStreaming::OxStream> stream;
    std::unique_ptr<Baumer::OXApi::Ox> ox;
    std::size_t count{0};
    std::float_t unit_ratio{1e-5}; // µm to m
    std::float_t measurement_distance{250e-3};

    ros::Publisher test_pub;

    void readSensorData(sensor_msgs::PointCloud2& cloud_msg) {
        // get the latest profile
        auto profile = ox->GetProfile();
        ++count;

        // put profile data into a PointCloud message:
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = profile.Length; //Number of points
        cloud.height = 1; // a single row, unordered point cloud
        cloud.is_dense = false; // Might contain NaN or Inf values, set to true if it doesn't
        cloud.points.resize(cloud.width * cloud.height);
        
        // sensor_msgs::ChannelFloat32 channel;
        // channel.name = "intensity";

        for (std::size_t i = 0; i < profile.Length; i++)
        {
            pcl::PointXYZ point;
            cloud.points[i].x = (profile.X.at(i)+profile.XStart)*unit_ratio;
            cloud.points[i].y = 0;

            std::uint16_t z(profile.Z.at(i));
            cloud.points[i].z = measurement_distance-z*unit_ratio;
        }
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp.fromNSec(ros::Time::now().toNSec());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_publisher");

    LaserScanPublisher lsp;

    return 0;
}
