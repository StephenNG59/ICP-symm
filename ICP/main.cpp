#include "stdafx.h"

#include "myicp.h"

using namespace std;

void help();

int main(int argc, char** argv)
{
	if (argc <= 3)
	{
		help();
		return 0;
	}

	// ----------------------------------------
	// --------1. symmetric registration-------
	// ----------------------------------------
	if (string(argv[1]) == "--symm")
	{
		MyICP myicp;

		// read files
		char deli[] = { '.' };
		string f1 = argv[2], f2 = argv[3];
		string ext1 = f1.substr(f1.find_last_of('.') + 1), ext2 = f2.substr(f2.find_last_of('.') + 1);
		if (ext1 == "pcd" && ext2 == "pcd")
		{
			myicp.LoadCloudPcd(f1, f2);
		}
		else if (ext1 == "obj" && ext2 == "obj")
		{
			myicp.LoadCloudObj(f1, f2);
		}
		else
		{
			cerr << "File extensions not correct!" << endl;
			help();
			return -1;
		}

		// params
		float diff_t = DEFAULT_DIFF_THRESH;
		int max_it = DEFAULT_MAX_ITERS;
		if (argc >= 5)
		{
			diff_t = atof(argv[4]);
			if (argc >= 6)
			{
				max_it = atoi(argv[5]);
			}
		}

		// register
		cout << diff_t << max_it << endl;
		Eigen::Affine3f guess = myicp.RegisterSymm(diff_t, max_it);

		// visualize
		myicp.Visualize();
	}

	// ----------------------------------------
	// --------2. transform--------------------
	// ----------------------------------------
	else if (string(argv[1]) == "-t")
	{
		if (argc != 11)
		{
			cerr << "Params not correct for transformation!" << endl;
			help();
			return -1;
		}

		// read files
		char deli[] = { '.' };
		string f1 = argv[2], f2 = argv[3];
		string ext1 = f1.substr(f1.find_last_of('.') + 1), ext2 = f2.substr(f2.find_last_of('.') + 1);
		pcl::PCLPointCloud2::Ptr pclcloud_src(new pcl::PCLPointCloud2), pclcloud_tgt(new pcl::PCLPointCloud2);
		pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>()), cloud_tgt(new pcl::PointCloud<PointT>());
		if (ext1 == "pcd")
		{
			pcl::PCDReader reader;
			reader.read(f1, *pclcloud_src);
			pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
		}
		else if (ext1 == "obj")
		{
			pcl::OBJReader reader;
			reader.read(f1, *pclcloud_src);
			pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
		}
		else
		{
			cerr << "File extensions not correct!" << endl;
			help();
			return -1;
		}

		// transform
		float ax = atof(argv[4]), ay = atof(argv[5]), az = atof(argv[6]), theta = atof(argv[7]) * M_PI / 180;
		float tx = atof(argv[8]), ty = atof(argv[9]), tz = atof(argv[10]);
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f(ax, ay, az).normalized()));
		transform.translation() << tx, ty, tz;
		pcl::transformPointCloud(*cloud_src, *cloud_tgt, transform);

		// save file
		if (ext2 == "pcd")
		{
			pcl::io::savePCDFile(f2, *cloud_tgt);
		}
		else if (ext2 == "obj")
		{
			cerr << "Can't save pointcloud into obj file!" << endl;
			help();
			return -1;
		}
		else
		{
			cerr << "File extensions not correct!" << endl;
			help();
			return -1;
		}
	}


	////transform
	//float theta = M_PI / 4.0;
	//Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	//transform.translation() << 2.5, 0.0, 0.0;
	//transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	//cout << "actual transform :" << endl
	//	<< transform.matrix() << endl;
	

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
	cout<< "--Usage                                                                                         --" << endl
		<< "--  --symm [source-pcd-file] [target-pcd-file] [optional: diff] [optional: max_iters]           --" << endl
		<< "--    use symmetric object function to register source to target pcd point clouds               --" << endl
		<< "--      e.g. ICP.exe --symm bunny.pcd bunny-out.pcd 0.1 50                                      --" << endl
		<< "--  -t [source-pcd-file] [output-pcd-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] --" << endl
		<< "--    transform source pcd file and save the result in output pcd file                          --" << endl
		<< "--      e.g. ICP.exe -t bunny.pcd bunny-out.pcd 0 0 1 30 2 2 -5                                 --" << endl
		<< endl;
}
