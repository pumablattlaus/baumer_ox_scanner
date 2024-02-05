import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def accumulate_point_clouds(bag_file, pc_topics=['/your_point_cloud_topic']):
    # Open the ROS bag file
    bag = rosbag.Bag(bag_file, 'r')

    # Initialize an empty array to hold all points
    all_points = np.empty((0, 3), dtype=np.float32)

    # Iterate through the bag file
    for topic, msg, t in bag.read_messages(topics=pc_topics):
        # Ensure the message is a PointCloud2 message
        # if isinstance(msg, PointCloud2):
        if msg._type == 'sensor_msgs/PointCloud2':
            # Convert PointCloud2 to an iterable of (x, y, z) points
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

            # Convert generator to a list of points and append to all_points
            points = np.array(list(gen))
            all_points = np.vstack((all_points, points))

    # # Create a final PointCloud2 message from all accumulated points
    # final_cloud = PointCloud2()
    # # You might need to set header and other fields for final_cloud as per your requirement
    # # final_cloud.header = ...

    # # Convert the numpy array of points back to a PointCloud2 message
    # final_cloud = pc2.create_cloud_xyz32(final_cloud.header, all_points.tolist())

    # Close the bag file
    bag.close()

    return all_points

if __name__ == '__main__':
    # Replace '/path/to/your/rosbag/file.bag' with the actual path to your ROS bag file
    bag_file_path = '/home/rosmatch/bags/tum/scanner_poses/in_base/pos4_quer_0.03.bag'
    all_points = accumulate_point_clouds(bag_file_path, pc_topics=['/cloud_out'])

    # Write the accumulated points to a file
    ## Get bag_file_path without the extension
    txt_file_name = bag_file_path.split('.bag')[0]+'.txt'
    np.savetxt(txt_file_name, all_points, delimiter=',')
