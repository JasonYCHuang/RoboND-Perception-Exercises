# Import PCL module
import pcl

# Voxel Grid filter.
def make_voxel(cloud, leaf_size = 0.01):
	vox = cloud.make_voxel_grid_filter()
	vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
	
	cloud_filtered = vox.filter()
	filename = 'voxel_downsampled.pcd'
	pcl.save(cloud_filtered, filename)
	return cloud_filtered


# PassThrough filter
def make_pass_through(cloud):
	pass_through = cloud.make_passthrough_filter()

	filter_axis = 'z'
	pass_through.set_filter_field_name(filter_axis)
	axis_min = 0.6
	axis_max = 1.1
	pass_through.set_filter_limits(axis_min, axis_max)

	cloud_filtered = pass_through.filter()
	filename = 'pass_through_filtered.pcd'
	pcl.save(cloud_filtered, filename)
	return cloud_filtered



# RANSAC plane segmentation
def make_ransac(cloud, max_dist = 0.01):
	seg = cloud.make_segmenter()

	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_model_type(pcl.SAC_RANSAC)
	seg.set_distance_threshold(max_dist)

	inliers, coefficients = seg.segment()
	return inliers, coefficients



# Extract inliers
def extract_inliers(cloud, inliers, negative = False):
	extracted_inliers = cloud.extract(inliers, negative=negative)
	filename = 'extracted_inliers.pcd'
	pcl.save(extracted_inliers, filename)
	return extracted_inliers

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers
def filter_outlier(cloud):
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(50)
	x = 1.0
	outlier_filter.set_std_dev_mul_thresh(x)
	cloud_filtered = outlier_filter.filter()
	filename = 'filter_outlier.pcd'
	pcl.save(cloud_filtered, filename)
	return cloud_filtered

# Save pcd for tabletop objects


if __name__ == "__main__":
	# Load Point Cloud file
	cloud = pcl.load_XYZRGB('tabletop.pcd')

	cloud_filtered_vox = make_voxel(cloud, 0.01)
	cloud_filtered_vox_pas = make_pass_through(cloud_filtered_vox)
	inliers, coefficients = make_ransac(cloud_filtered_vox_pas, 0.01)
	extracted_inliers = extract_inliers(cloud_filtered_vox_pas, inliers, True)
	cloud_filtered_vox_pas_out = filter_outlier(extracted_inliers)

