#include <iostream>

#include <pcl/io/pcd_io.h>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include "regist.h"


int main(int argc, char **argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// load source
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cat.pcd", *cloud_in) == -1)
	{
		PCL_ERROR("Couldn't read file\n");
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("cat_out.pcd", *cloud_out) == -1)
	{
		PCL_ERROR("Couldn't read file\n");
		return -1;
	}


	/*// transform
	float theta = M_PI / 4.0;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 2.5, 0.0, 0.0;
	transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

	// visualize
	pcl::visualization::PCLVisualizer viewer("test");
	viewer.addPointCloud(cloud_out);
	
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	// save file
	pcl::io::savePCDFile("cat_out.pcd", *cloud_out);*/


	// test the registration function
	std::vector<cv::Point3d> src, dst;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		src.push_back(cv::Point3d(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z));
		dst.push_back(cv::Point3d(cloud_out->points[i].x, cloud_out->points[i].y, cloud_out->points[i].z));
	}

	cv::Mat R, T;
	registrateNPoint(src, dst, R, T);

	cout << "R" << R << endl;
	cout << "T" << T << endl;
}