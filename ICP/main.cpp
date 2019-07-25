#include "stdafx.h"

#include "myicp.h"
#include "func.h"

using namespace std;

void help();

int main(int argc, char** argv)
{
	if (argc <= 3)
	{
		help();
		return 0;
	}

	if (string(argv[1]) == "--symm")
	{	// ----------------------------------------
		// --------1. symmetric registration-------
		// ----------------------------------------
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
		bool showOnce = false;
		float diff_t = DEFAULT_DIFF_THRESH;
		int max_it = DEFAULT_MAX_ITERS;
		if (argc >= 5)
		{
			if (string(argv[4]) == "--show-once")
				showOnce = true;
			else
			{
				max_it = atof(argv[4]);
				if (argc >= 6)
				{
					if (string(argv[5]) == "--show-once")
						showOnce = true;
					else
						diff_t = atoi(argv[5]);
				}
			}
		}

		// register
		Eigen::Affine3f guess = myicp.RegisterSymm(diff_t, max_it);

		// visualize
		myicp.Visualize(showOnce);
	}
	else if (string(argv[1]) == "--p2p")
	{
		// read files
		pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>()), cloud_tgt(new pcl::PointCloud<PointT>()), cloud_guess(new pcl::PointCloud<PointT>());
		
		pcl::PCDReader reader;
		reader.read(string(argv[2]), *cloud_src);
		reader.read(string(argv[3]), *cloud_tgt);

		// get params
		int max_iters = 500;
		bool showOnce = false;
		if (argc >= 5)
		{
			if (string(argv[4]) == "--show-once")
				showOnce = true;
			else
			{
				max_iters = atoi(argv[4]);
				if (argc >= 6 && string(argv[5]) == "--show-once")
					showOnce = true;
			}
		}

		// icp
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(cloud_src);
		icp.setInputTarget(cloud_tgt);
		icp.setMaximumIterations(max_iters);
		icp.setEuclideanFitnessEpsilon(0.0001);
		icp.align(*cloud_guess);
		Eigen::Matrix4f transform = icp.getFinalTransformation();

		// print result
		cout << "Final transform" << endl << transform << endl;

		// visualize
		pcl::visualization::PointCloudColorHandlerCustom<PointT>
			red_handler(cloud_src, 200, 20, 20), green_handler(cloud_tgt, 20, 200, 20), blue_handler(cloud_guess, 20, 20, 200);
		pcl::visualization::PCLVisualizer viewer("Viewer#1");
		viewer.setBackgroundColor(255, 255, 255);
		viewer.addPointCloud(cloud_src, red_handler, "cloud1");
		viewer.addPointCloud(cloud_tgt, green_handler, "cloud2");
		viewer.addPointCloud(cloud_guess, blue_handler, "cloud3");

		if (showOnce)
			viewer.spinOnce();
		else
			while (!viewer.wasStopped()) viewer.spinOnce();
	}
	else if (string(argv[1]) == "-t" || string(argv[1]) == "-td" || string(argv[1]) == "-tdh")
	{	// ----------------------------------------
		// --------2. transform--------------------
		// ----------------------------------------

		if ((string(argv[1]) == "-t" && argc != 11) 
			|| ((string(argv[1]) == "-td" || string(argv[1]) == "-tdh") && argc != 12))
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

		// delete some points
		if (string(argv[1]) == "-td")
		{
			deleteSomePoints(cloud_tgt, atof(argv[11]), false);
		}
		else if (string(argv[1]) == "-tdh")
		{
			deleteSomePoints(cloud_tgt, atof(argv[11]), true);
		}

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

		// print transform matrix
		cout << "Transformation suceeds!" << endl << transform.matrix() << endl;

	}
	else if (string(argv[1]) == "-cx" || string(argv[1]) == "-cy" || string(argv[1]) == "-cz")
	{	// ----------------------------------------
		// --------3. cut out some points----------
		// ----------------------------------------
		string f1 = string(argv[2]);
		float xyz_ = atof(argv[4]);
		pcl::PCLPointCloud2::Ptr pclcloud_src(new pcl::PCLPointCloud2);
		pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>());
		pcl::PCDReader reader;
		reader.read(f1, *pclcloud_src);
		pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
		if (string(argv[1]) == "-cx")
		{
			for (int i = 0; i < cloud_src->points.size(); i++)
			{
				if (cloud_src->points[i].x > xyz_)
				{
					cloud_src->points.erase(cloud_src->points.begin() + i);
					i--;
				}
			}
		}
		else if (string(argv[1]) == "-cy")
		{
			for (int i = 0; i < cloud_src->points.size(); i++)
			{
				if (cloud_src->points[i].y > xyz_)
				{
					cloud_src->points.erase(cloud_src->points.begin() + i);
					i--;
				}
			}
		}
		else if(string(argv[1]) == "-cz")
		{
			for (int i = 0; i < cloud_src->points.size(); i++)
			{
				if (cloud_src->points[i].z > xyz_)
				{
					cloud_src->points.erase(cloud_src->points.begin() + i);
					i--;
				}
			}
		}
		
		cloud_src->width = cloud_src->points.size();		//! remember to do this
		pcl::io::savePCDFile(string(argv[3]), *cloud_src);
	}

	else
	{
		cerr << "Wrong option!" << endl;
		return -1;
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
	cout << "------------------------------------------------------------------------------------------------" << endl
		<< "------------------------------------------[ Options ]-------------------------------------------" << endl
		//<< "------------------------------------------------------------------------------------------------" << endl
		<< "  --symm [source-file] [target-file] [optional: max_iters] [optional: diff]                     " << endl
		<< "      use symmetric object function to REGISTER source to target point clouds                   " << endl
		<< "  -t [source-file] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z]           " << endl
		<< "      TRANSFORM source file and save the result in output pcd file                              " << endl
		<< "  -td [source] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] [ratio]       " << endl
		<< "  -tdh [source] [output-file] [axis-x, y, z] [theta-in-degree] [translate-x, y, z] [ratio]      " << endl
		<< "      TRANSFORM, meanwhile DELETE *ratio* of the origin points.                                 " << endl
		<< "      *-td* for randomly deletion, *-tdh* for hard deletion of the front part                   " << endl
		<< "  -cx/cy/cz [source] [output-file] xyz_cut                                                      " << endl
		<< "      CUT OUT the points whose x/y/z coord is greater than *xyz_cut*                            " << endl
		<< "------------------------------------------------------------------------------------------------" << endl
		<< "    e.g. >ICP.exe --symm bunny-src.pcd bunny-tgt.pcd 50 0.001                                   " << endl
		<< "    e.g. >ICP.exe --p2p bunny-src.pcd bunny-tgt.pcd 10 --show-once                              " << endl
		<< "    e.g. >ICP.exe -t bunny.pcd bunny-src-trans.pcd 1 0 1 30 10 -10 -5                           " << endl
		<< "    e.g. >ICP.exe -td bunny.pcd bunny-src-delet.pcd 0 0 1 30 2 2 -5 0.4                         " << endl
		<< "    e.g. >ICP.exe -cy bunny.pcd bunny-cuty.pcd 2     # if y > 2, delete this point              " << endl
		<< "------------------------------------------------------------------------------------------------" << endl
		<< endl;
}
