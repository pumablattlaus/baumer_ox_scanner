import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudToPointCloud2:
    def __init__(self, sub_topic='/cloud_accumulated', pub_topic='cloud_high_rate'):
        self.point_cloud_sub = rospy.Subscriber(sub_topic, PointCloud2, self.set_cloud)
        self.point_cloud2_pub = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)
        self.point_cloud = PointCloud2()
        self.rate = rospy.Rate(50)
        self.run()

    def set_cloud(self, old_point_cloud):
        self.point_cloud = old_point_cloud
        
    def run(self):
        while not rospy.is_shutdown():
            self.point_cloud.header.stamp = rospy.Time.now()
            self.point_cloud2_pub.publish(self.point_cloud)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('point_cloud_to_high_rate')
    PointCloudToPointCloud2()