#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>

class PointCloudAssembler {
public:
    PointCloudAssembler() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Get cloud frame from parameter server
        nh.param<std::string>("cloud_frame", cloud_accumulated_frame, "mur620a/base_link"); //TODO: Use in transformation in the end

        // Set time interval for throtteling
        time_interval = 0.2;

        // Initialize time stamp for throtteling
        last_time = ros::Time(0);

        // Subscriber to point cloud topic
        cloud_sub = nh.subscribe("/cloud_out", 10, &PointCloudAssembler::cloudCallback, this);

        // Publisher for the accumulated point cloud
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_accumulated", 10, true);

        // Services to start and stop point cloud assembly
        start_service = nh.advertiseService("start", &PointCloudAssembler::startAssembly, this);
        finish_service = nh.advertiseService("finish", &PointCloudAssembler::finishAssembly, this);

        // Initialize assembling flag
        assembling = false;
    }

    bool startAssembly(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        assembling = true;
        assembled_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        res.success = true;
        res.message = "Assembly started.";
        return true;
    }

    bool finishAssembly(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        if (assembling && assembled_cloud) {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*assembled_cloud, output);
            output.header.stamp = ros::Time::now();
            output.header.frame_id = cloud_frame; //TODO: Use in transformation in the end
            cloud_pub.publish(output);
            res.success = true;
            res.message = "Assembly finished and published.";
        } else {
            res.success = false;
            res.message = "No point clouds were assembled.";
        }
        assembling = false;
        return true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
        ros::Time current_time = ros::Time::now();
        if (assembling && (current_time - last_time).toSec() > time_interval) {
            last_time = current_time;
            pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
            pcl::fromROSMsg(*input_cloud, temp_cloud);
            *assembled_cloud += temp_cloud;
            cloud_frame = input_cloud->header.frame_id; //has to always be the same
        }
    }

private:
    double time_interval;
    std::string cloud_accumulated_frame;
    std::string cloud_frame;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;
    ros::ServiceServer start_service, finish_service;
    bool assembling;
    ros::Time last_time;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled_cloud;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_assembler");
    PointCloudAssembler assembler;
    ros::spin();
    return 0;
}
