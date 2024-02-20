import numpy as np
import tf.transformations as tf_trans
import rosbag


def get_transformation_matrix(pose):
    # Extract translation and rotation from the pose
    translation = [pose.position.x, pose.position.y, pose.position.z]
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Create transformation matrix from translation and rotation
    t_matrix = tf_trans.concatenate_matrices(tf_trans.translation_matrix(translation), tf_trans.quaternion_matrix(rotation))
    return t_matrix

def transform_points(points, t_matrix):
    transformed_points = []
    for point in points:
        # Convert point to homogeneous coordinates
        point_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        # Apply transformation
        transformed_point_homogeneous = np.dot(t_matrix, point_homogeneous)
        # Convert back to 3D coordinates
        transformed_point = transformed_point_homogeneous[:3]
        transformed_points.append(transformed_point)
    return np.array(transformed_points)

def get_transformation_from_bag(bag, pose_topic='/robot_pose'):
    pose = None

    # Iterate through the bag file
    for topic, msg, t in bag.read_messages(topics=[pose_topic]):
        if msg._type == 'geometry_msgs/Pose':
            pose=msg
        elif msg._type == 'geometry_msgs/PoseStamped':
            pose=msg.pose

    # Close the bag file
    bag.close()

    if pose is None:
        raise Exception('No pose message found in the bag file')
    t_matrix=get_transformation_matrix(pose)
    return t_matrix

def transform_points_from_bag(bag_file, points, pose_topic='/robot_pose'):

    # Get transformation matrix from the bag file
    t_matrix = get_transformation_from_bag(bag_file, pose_topic)

    # Transform the points
    transformed_points = transform_points(points, t_matrix)

    return transformed_points