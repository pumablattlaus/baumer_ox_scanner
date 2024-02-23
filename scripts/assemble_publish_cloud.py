#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerResponse

class PointCloudAssembler:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('point_cloud_assembler')

        # Create a subscriber to the point cloud topic
        self.sub = rospy.Subscriber('/cloud_out', PointCloud2, self.cloud_callback)

        # Create a publisher for the accumulated point cloud
        self.pub = rospy.Publisher('/cloud_accumulated', PointCloud2, queue_size=10)

        # Services to start and stop point cloud assembly
        self.start_service = rospy.Service('start', Trigger, self.start_assembly)
        self.finish_service = rospy.Service('finish', Trigger, self.finish_assembly)

        # Variable to store the assembled point cloud
        self.assembled_cloud = None
        self.cloud_frame = "assembled_cloud"

        # Flag to control the assembly process
        self.assembling = False

    def cloud_callback(self, msg):
        # Callback for processing incoming point cloud messages
        if self.assembling:
            if self.assembled_cloud is None:
                self.assembled_cloud = pcl.PointCloud_PointXYZRGB()
                pcl.fromROSMsg(msg, self.assembled_cloud)
            else:
                cloud = pcl.PointCloud_PointXYZRGB()
                pcl.fromROSMsg(msg, cloud)
                self.assembled_cloud += cloud

            self.cloud_frame = msg.header.frame_id

    def start_assembly(self, req):
        # Service callback to start the assembly process
        self.assembling = True
        self.assembled_cloud = None
        return TriggerResponse(success=True, message="Assembly started.")

    def finish_assembly(self, req):
        # Service callback to stop the assembly process and publish the result
        self.assembling = False
        if self.assembled_cloud is not None:
            assembled_msg = pcl.toROSMsg(self.assembled_cloud)
            assembled_msg.header.stamp = rospy.Time.now()
            assembled_msg.header.frame_id = self.cloud_frame
            self.pub.publish(assembled_msg)
            return TriggerResponse(success=True, message="Assembly finished and published.")
        else:
            return TriggerResponse(success=False, message="No point clouds were assembled.")

if __name__ == '__main__':
    assembler = PointCloudAssembler()
    rospy.spin()
