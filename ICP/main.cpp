#include "stdafx.h"

#include "myicp.h"
#include "func.h"

using namespace std;

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

void getSymmParams(int argc, char** argv, bool& showOnce, int& maxIters, float& diffThreshold, int& guessTimes)
{
	if (argc >= 5)
	{
		if (string(argv[4]) == "--show-once")
			showOnce = true;
		else
		{
			maxIters = atof(argv[4]);
			if (argc >= 6)
			{
				if (string(argv[5]) == "--show-once")
					showOnce = true;
				else
					diffThreshold = atoi(argv[5]);
			}
		}
	}
	if (string(argv[1]).length() > 6)
	{
		guessTimes = atoi(argv[1] + 6);		//? works?
	}
}

void getP2pParams(int argc, char** argv, bool& showOnce, int& maxIters)
{
	if (argc >= 5)
	{
		if (string(argv[4]) == "--show-once")
			showOnce = true;
		else
		{
			maxIters = atoi(argv[4]);
			if (argc >= 6 && string(argv[5]) == "--show-once")
				showOnce = true;
		}
	}
}

void runPCAtest()
{
	float p[5][3] = {
			{0, 7, 2},
			{0, 1, 0},
			{0, -1, 0},
			{0, 2, 0},
			{0, -2, 0},
	};

	float q[5][3] = {
		{7, 0, 2},
		{1, 0, 0},
		{-1, 0, 0},
		{2, 0, 0},
		{-2, 0, 0},
	};

	Eigen::MatrixXf P(5, 3), Q(5, 3);
	for (size_t i = 0; i < 5; i++)
	{
		for (size_t j = 0; j < 3; j++)
		{
			P(i, j) = p[i][j];
			Q(i, j) = q[i][j];
		}
	}

	vector<Eigen::Vector4f> eig_p, eig_q;
	get3Dpca(P, eig_p);
	get3Dpca(Q, eig_q);

	Eigen::Affine3f r = getRotateMatrix(eig_p, eig_q);

	cout << "r" << endl << r.matrix() << endl;
}

int main(int argc, char** argv)
{
	srand((int)time(0));

	if (argc <= 1)
	{
		help();
		return 0;
	}

	if (string(argv[1]).substr(0, 6) == "--symm" || string(argv[1]) == "--p2p" || string(argv[1]) == "--pca")
	{
		/* ---------- registration : symmectric / point-to-point(umeyama) --------- */

		MyICP myicp;

		// read files
		if (myicp.LoadCloudFiles(string(argv[2]), string(argv[3])) == -1)
		{
			cerr << "Cannot load files!" << endl;
			return -1;
		}

		// params
		bool showOnce = false;
		int maxIters = DEFAULT_MAX_ITERS, guessTimes = DEFAULT_GUESS_TIMES;
		float diffThreshold = DEFAULT_DIFF_THRESH;

		// get params and register
		if (string(argv[1]).substr(0, 6) == "--symm" || string(argv[1]) == "--pca")
		{
			// get params
			getSymmParams(argc, argv, showOnce, maxIters, diffThreshold, guessTimes);
			myicp.SetSymmParams(maxIters, diffThreshold, guessTimes);
			myicp.SetUsePca(string(argv[1]) == "--pca");
			
			// register
			Eigen::Affine3f guess = myicp.RegisterSymm();
			cout << "Guess transform:\n" << guess.matrix() << endl;
		}
		else if (string(argv[1]) == "--p2p")
		{
			// get params
			getP2pParams(argc, argv, showOnce, maxIters);
			myicp.SetP2pParams(maxIters);

			// register
			Eigen::Affine3f guess = myicp.RegisterP2P();
			cout << "Guess transform:\n" << guess.matrix() << endl;
		}

		// visualize
		myicp.Visualize(showOnce);
	}
	else if (string(argv[1]) == "-t" || string(argv[1]) == "-td" || string(argv[1]) == "-tdh")
	{
		/* ----------------- transform ------------------- */

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
			cerr << "File type not supported!" << endl;
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
	{	
		/* ----------------- cut out some points ------------------- */

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

	return 0;
}

