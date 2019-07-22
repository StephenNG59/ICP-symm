#include "stdafx.h"

#include "myicp.h"

int main(int argc, char** argv)
{
	MyICP myicp;
	myicp.LoadCloud("cat.pcd", "cat_out.pcd");
	
	myicp.RegisterSymm();

	// visualize
	pcl::visualization::PCLVisualizer viewer("test");
	// translate to PointXYZ type so that can be added into viewer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<PointT>::Ptr cloud = myicp.GetSrcCloud();
	int npts = cloud->points.size();
	for (size_t i = 0; i < npts; i++)
	{
		pcl::PointXYZ p;
		p.x = cloud->points[i].x, p.y = cloud->points[i].y, p.z = cloud->points[i].z;
		cloud1->points.push_back(p);
	}
	cloud = myicp.GetTgtCloud();
	npts = cloud->points.size();
	for (size_t i = 0; i < npts; i++)
	{
		pcl::PointXYZ p;
		p.x = cloud->points[i].x, p.y = cloud->points[i].y, p.z = cloud->points[i].z;
		cloud2->points.push_back(p);
	}
	//viewer.addPointCloud(cloud1);
	viewer.addPointCloud(cloud2);

	// show viewer
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


	// //transform
	//float theta = M_PI / 4.0;
	//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform.translation() << 2.5, 0.0, 0.0;
	//transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	//pcl::transformPointCloud(*(myicp.GetSrcCloud()), *(myicp.GetTgtCloud()), transform);

	// //save file
	//pcl::io::savePCDFile("cat_out.pcd", *(myicp.GetTgtCloud()));


	/*// test the registration function
	std::vector<cv::Point3d> src, dst;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		src.push_back(cv::Point3d(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z));
		dst.push_back(cv::Point3d(cloud_out->points[i].x, cloud_out->points[i].y, cloud_out->points[i].z));
	}

	cv::Mat R, T;
	registrateNPoint(src, dst, R, T);

	cout << "R" << R << endl;
	cout << "T" << T << endl;*/
}