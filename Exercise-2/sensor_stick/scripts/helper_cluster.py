import pcl
from pcl_helper import *

def kd_tree_clustering(white_cloud, tree, size_min = 10, size_max = 4000, tolerance = 0.05):
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(tolerance)
    ec.set_MinClusterSize(size_min)
    ec.set_MaxClusterSize(size_max)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices

def color_cluster(white_cloud, tree):
    cluster_indices = kd_tree_clustering(white_cloud, tree)
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                white_cloud[indice][0], 
                white_cloud[indice][1], 
                white_cloud[indice][2], 
                rgb_to_float(cluster_color[j])
            ])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud