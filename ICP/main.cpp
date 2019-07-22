#include "stdafx.h"

#include "myicp.h"

void help();

int main(int argc, char** argv)
{
	help();

	MyICP myicp;
	myicp.LoadCloud("cat.pcd", "cat_out.pcd");
	pcl::PointCloud<PointT>::Ptr cloud_src = myicp.GetSrcCloud(), cloud_tgt = myicp.GetTgtCloud(), cloud_apply(new pcl::PointCloud<PointT>);
	
	// registrate
	Eigen::Affine3f guess = myicp.RegisterSymm();

	 //transform
	float theta = M_PI / 4.0;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 2.5, 0.0, 0.0;
	transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	cout << "actual transform :" << endl
		<< transform.matrix() << endl;
	// apply 
	pcl::transformPointCloud(*cloud_src, *cloud_apply, guess);

	// visualize
	pcl::visualization::PointCloudColorHandlerCustom<PointT> 
		red_handler(cloud_src, 200, 20, 20), green_handler(cloud_tgt, 20, 200, 20), blue_handler(cloud_apply, 20, 20, 200);
	pcl::visualization::PCLVisualizer viewer("Viewer#1");
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(cloud_src, red_handler, "cloud1");
	viewer.addPointCloud(cloud_tgt, green_handler, "cloud2");
	viewer.addPointCloud(cloud_apply, blue_handler, "cloud3");


	// show viewer
	//while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

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

void help()
{
	cout << "Usage:" << endl
		<< "  --symm [source-pcd-file] [target-pcd-file] :" << endl
		<< "    use symmetric object function to registrate 2 pcd point clouds" << endl
		<< "  -v [source-pcd-file] [output-pcd-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] :" << endl
		<< "    transform source pcd file and save the result in output pcd file" << endl
		<< "      e.g. \".\\ICP.exe\" -v \".\\bunny.pcd\" \".\\bunny-out.pcd\" 0 0 1 30 2 2 -5" << endl
		<< endl;
}
