#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud.h>
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
        int udpPort;
        nh.param<std::string>("host", host, "192.168.0.250");
        nh.param<int>("udp_port", udpPort, 12345);
        bool udp; // use UDP protocol
        nh.param<bool>("udp", udp, false);

        // Create a publisher object
        // laser_scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
        test_pub = nh.advertise<std_msgs::Float32>("test", 10);

        if (udp){
            ox = Baumer::OXApi::Ox::Create( host, udpPort );
            // Initialize sensor and any other necessary variables
            // create an instance of a OxStream object
            stream = ox->CreateStream();

            // set socket buffer size, needs to be set large enough so that no udp
            // packets get lost
            stream->SetReceiveBufferSize( 2 * 1024 * 1024 );
            stream->Start();
            ROS_INFO_STREAM("Listening on udp port " << udpPort << ".\n");
        }
        else {
            // Initialize sensor and any other necessary variables
            ox = Baumer::OXApi::Ox::Create( host );
            // opens the network connection
            ox->Connect();
            ROS_INFO_STREAM("Connected to " << host << ".\n");
            ROS_INFO("NOT using UDP.");
        }
        // Main loop to read and publish laser scan data
        ros::Rate loop_rate(400); // 50 Hz, adjust as necessary
        while (ros::ok() && !gSignalStatus) {
            sensor_msgs::PointCloud point_msg;
            // Fill in PointCloud message
            // For example:
            // point_msg.header.stamp = ros::Time::now();
            point_msg.header.frame_id = "laser_scanner";
            // Fill in other necessary fields such as angle_min, angle_max, range_min, range_max, etc.
            // ...

            // Read data from the sensor
            if (udp) {
                if (readSensorDataUDP(point_msg)) {
                    // Publish the laser scan data
                    point_cloud_pub.publish(point_msg);
                }
            }
            else {
                readSensorData(point_msg);
                point_cloud_pub.publish(point_msg);
            }

            // ros::spinOnce();
            loop_rate.sleep();
        }

        // Disconnect from the sensor
        if( udp )
        {
            stream->Close();
        }
        else
        {
            ox->Disconnect();
        }
    }

private:
    ros::Publisher laser_scan_pub;
    ros::Publisher point_cloud_pub;
    std::shared_ptr<Baumer::OXApi::UdpStreaming::OxStream> stream;
    std::unique_ptr<Baumer::OXApi::Ox> ox;
    std::size_t count{0};
    std::float_t unit_ratio{1e-5}; // µm to m
    std::float_t measurement_distance{250e-3};

    ros::Publisher test_pub;

    bool readSensorDataUDP(sensor_msgs::PointCloud& cloud_msg) {
        // Use your sensor's API to read the data
        // For example:
        // auto sensor_data = stream->ReadProfile();
        // Convert sensor_data to scan ranges
        // ...
        if( stream->ProfileAvailable( ) )
            {

                std::float_t test{0};

                auto profile = stream->ReadProfile( );
                ++count;
                std::uint64_t timestamp(profile.TimeStamp);
                
                std::vector<geometry_msgs::Point32> point(profile.Length);
                // std::vector<sensor_msgs::ChannelFloat32> channels(profile.Length);
                sensor_msgs::ChannelFloat32 channel;
                channel.name = "intensity";

                // ROS_INFO_STREAM("Profile " << count << " BlockId: " << profile.BlockId << " Length: " << profile.Length << " Timestamp: " << timestamp);
                for (std::size_t i = 0; i < profile.Length; i++)
                {
                    point.at(i).x = profile.X.at(i)*unit_ratio;
                    point.at(i).y = 0;

                    if (profile.ZValid)
                    {
                        std::uint16_t z(profile.Z.at(i));
                        point.at(i).z = z*unit_ratio; //mm to m

                        
                        if(z != std::numeric_limits<std::uint16_t>::quiet_NaN() && z != std::numeric_limits<std::uint16_t>::signaling_NaN())
                        {
                            test += z*unit_ratio;
                        }
                    }
                    else
                    {
                        point.at(i).z = 0;
                    }
                    if (profile.IntensityValid)
                    {
                        std::uint16_t intensity(profile.I.at(i));
                        channel.values.push_back(intensity);
                    }
                    else
                    {
                        channel.values.push_back(0);
                    }
                    
                }
                // cloud_msg.header.stamp.fromNSec(timestamp);
                cloud_msg.header.stamp.fromNSec(ros::Time::now().toNSec());
                cloud_msg.points = point;
                cloud_msg.channels.push_back(channel);

                test /= profile.Length;
                std_msgs::Float32 test_msg;
                test_msg.data = test;
                test_pub.publish(test_msg);
            }
            
        // check for errors
            if( stream->ErrorOccured( ) )
            {
                auto error = stream->ReadError( );
                ROS_ERROR_STREAM("Error BlockId: " << error.BlockId << " Message: " << error.Message);
                return false;
            }

        // ROS_INFO_STREAM("Read " << cloud_msg.points.size() << " points.");
        return true; // Return false if there was an error
    }

    void readSensorData(sensor_msgs::PointCloud& cloud_msg) {
        // get the latest profile
        auto profile = ox->GetProfile();
        ++count;

        // put profile data into a PointCloud message:
        std::vector<geometry_msgs::Point32> points(profile.Length);
        
        // sensor_msgs::ChannelFloat32 channel;
        // channel.name = "intensity";

        for (std::size_t i = 0; i < profile.Length; i++)
        {
            points.at(i).x = (profile.X.at(i)+profile.XStart)*unit_ratio;
            points.at(i).y = 0;

            std::uint16_t z(profile.Z.at(i));
            points.at(i).z = measurement_distance-z*unit_ratio;
        }
        cloud_msg.header.stamp.fromNSec(ros::Time::now().toNSec());
        cloud_msg.points = points;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_publisher");

    LaserScanPublisher lsp;

    return 0;
}
