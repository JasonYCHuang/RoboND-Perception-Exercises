#!/usr/bin/env python

import numpy as np
import sklearn

from visualization_msgs.msg import Marker

from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.pcl_helper import *

from helper_segmentation import *
from helper_clustering import *
from helper_recognition import *

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    cloud_table, cloud_objects = execute_segmentation(pcl_msg)
    white_cloud, cluster_cloud, cluster_indices = execute_clustering(cloud_objects)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud_objects = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud_objects)

# Exercise-3 TODOs: 
    detected_objects = execute_recognition(
        clf, 
        scaler, 
        encoder,
        object_markers_pub,
        cloud_objects, 
        cluster_indices,
        white_cloud
    )

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

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

    object_markers_pub = rospy.Publisher(
        '/object_markers', 
        Marker, 
        queue_size=1
    )

    detected_objects_pub = rospy.Publisher(
        '/detected_objects', 
        DetectedObjectsArray, 
        queue_size=1
    )

    # TODO: Load Model From disk
    clf, encoder, scaler = load_prediction_model()

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()


