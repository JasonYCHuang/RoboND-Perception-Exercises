#!/usr/bin/env python

# Import modules
from pcl_helper import *
from helper_ransac import *
from helper_cluster import *

# TODO: Define functions as required


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    cloud_vx = make_voxel(cloud)

    # TODO: PassThrough Filter
    cloud_vx_ps = make_pass_through(cloud_vx)

    # TODO: RANSAC Plane Segmentation
    inliers, coefficients = make_ransac(cloud_vx_ps)

    # TODO: Extract inliers and outliers
    cloud_table = extract_inliers(cloud_vx_ps, inliers, False)
    cloud_objects = extract_inliers(cloud_vx_ps, inliers, True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = color_cluster(white_cloud, tree)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud_objects = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud_objects)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber(
        '/sensor_stick/point_cloud', 
        pc2.PointCloud2, 
        pcl_callback, 
        queue_size=1
    )

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher(
        '/pcl_objects', 
        PointCloud2, 
        queue_size=1
    )

    pcl_table_pub = rospy.Publisher(
        '/pcl_table', 
        PointCloud2, 
        queue_size=1
    )

    pcl_cluster_pub = rospy.Publisher(
        '/pcl_cluster', 
        PointCloud2, 
        queue_size=1
    )
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

