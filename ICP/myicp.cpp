#include "stdafx.h"

#include "myicp.h"
#include "func.h"

MyICP::MyICP() : maxIters(DEFAULT_MAX_ITERS), diffThreshold(DEFAULT_DIFF_THRESH), guess_times()
{
	cloud_src = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_tgt = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_src_demean = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_tgt_demean = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_apply = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_pn_src_demean = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	cloud_pn_tgt_demean = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	cloud_pn_med_demean = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
}

MyICP::~MyICP()
{
}

// ---------------------

int MyICP::LoadCloudFiles(std::string src_path, std::string tgt_path)
{
	pcl::PCDReader pcdReader;
	pcl::OBJReader objReader;
	
	std::string ext1 = src_path.substr(src_path.find_last_of('.') + 1);
	if (ext1 == "pcd")
	{
		if (pcdReader.read(src_path, *cloud_src) < 0) return -1;
	}
	else if (ext1 == "obj")
	{
		if (objReader.read(src_path, *cloud_src) < 0) return -1;
	}
	else return -1;

	std::string ext2 = tgt_path.substr(tgt_path.find_last_of('.') + 1);
	if (ext2 == "pcd")
	{
		if (pcdReader.read(tgt_path, *cloud_tgt) < 0) return -1;
	}
	else if (ext2 == "obj")
	{
		if (objReader.read(tgt_path, *cloud_tgt) < 0) return -1;
	}
	else return -1;

	cout << "[-] source pts: " << cloud_src->width << " || target pts: " << cloud_tgt->width << endl;

	return 0;
}

void MyICP::Visualize(bool showOnce /*= false*/)
{
	pcl::transformPointCloud(*cloud_src, *cloud_apply, guess_transform);

	pcl::visualization::PointCloudColorHandlerCustom<PointT>
		red_handler(cloud_src, 200, 20, 20), green_handler(cloud_tgt, 20, 200, 20), blue_handler(cloud_apply, 20, 20, 200);
	pcl::visualization::PCLVisualizer viewer("Viewer#1");
	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(cloud_src, red_handler, "cloud1");
	viewer.addPointCloud(cloud_tgt, green_handler, "cloud2");
	viewer.addPointCloud(cloud_apply, blue_handler, "cloud3");

	if (showOnce)
		viewer.spinOnce();
	else
		while (!viewer.wasStopped()) viewer.spinOnce();
}

// ---------------------

void MyICP::SetSymmParams(int maxIters, float diffThreshold, int guessTimes)
{
	this->maxIters = maxIters, this->diffThreshold = diffThreshold, this->guess_times = guessTimes, this->maxCounts = 0.5 * maxIters;
}

Eigen::Affine3f MyICP::RegisterSymm()
{
	assert(cloud_src && cloud_tgt);
	float cloud_size = cloud_src->getMatrixXfMap().maxCoeff() - cloud_src->getMatrixXfMap().minCoeff();
	float diffTScale = this->diffThreshold * cloud_size;

	// 0. demean (XYZ -> demean)
	demean();

	// 1. estimate normals (use demean XYZ, concatenate with normal)
	estimateNormals();


	// 2. copy demean src & tgt cloud into eigen matrix
	pasteInMatrix(this->cloud_pn_src_demean, this->src_mat_xyz, this->src_mat_normal);
	pasteInMatrix(this->cloud_pn_tgt_demean, this->tgt_mat_xyz, this->tgt_mat_normal);


	// 3. initialize correspondences estimation
	initCorrespondenceEstimation();


	// 4. initialize transform
	Eigen::Affine3f demean_transform = getInitTransform();
	

	#pragma region 5. iterates until convergence / max
	// 5. iterates until convergence / max
	int iters = 0, count = 0;
	float diff = diffTScale + 1;
	while (diff > diffTScale && iters < this->maxIters)
	{
		// 5.1. find correspondence
		findCorrespondences(this->ce, this->cloud_pn_med_demean, this->cloud_pn_tgt_demean, this->correspondences);

		float cor_ratio = float(this->correspondences.size()) / this->cloud_src->width;
		if (cor_ratio < MED_COR_RATIO)
		{
			//todo relax rejection...
			if (cor_ratio < MIN_COR_RATIO)
			{
				cout << "Too few points for correspondence, full correspondence triggered: " << correspondences.size() << endl;
				findCorrespondences(this->ce, this->cloud_pn_med_demean, this->cloud_pn_tgt_demean, this->correspondences, false);//? 为什么一用到这个，就会发癫一样平移……
			}
		}
		/*findCorrespondences(cloud_pn_med_demean, cloud_pn_tgt_demean, correspondences, true);
		if (correspondences.size() == 0)
		{
			findCorrespondences(cloud_pn_med_demean, cloud_pn_tgt_demean, correspondences, false);
			cout << "[*] WARNING: point sets may not converge!" << endl;
		}*/
		
		// 5.2. paste into new matrix based on correspondences
		pasteWithCorrespondence(correspondences, src_mat_xyz, src_mat_xyz_cor, tgt_mat_xyz, tgt_mat_xyz_cor);
		pasteWithCorrespondence(correspondences, src_mat_normal, src_mat_normal_cor, tgt_mat_normal, tgt_mat_normal_cor);

		// 5.3. evaluate diff, may break
		float next_diff = evalDiff(src_mat_xyz_cor, tgt_mat_xyz_cor);
		if (next_diff < diffTScale)
		{
			diff = next_diff;
			break;
		}
		if (next_diff > diff)
		{
			count += iters / 8;										// punishment for negative-progress will increase
		}
		else
		{
			count += std::min(5.0f, (diffTScale / 50) / (diff - next_diff));		// as the progress becomes smaller, the count speed becomes more rapidly
			if (count > this->maxCounts)
			{
				cerr << "[*] WARNING: seems like this cannot progress anymore..." << endl;
				break;
			}
		}
		diff = next_diff;
		if (iters == 0 || iters % 10 == 1)
			cout << "[ ] iters#" << std::right << setw(3) << iters << " - diff: " << diff / cloud_size * 1000 << "‰" << endl;
		iters++;

		// 5.4. estimate best transform from src_cor to tgt_cor
		Eigen::Affine3f incre_transform = estimateTransformSymm(src_mat_xyz_cor, src_mat_normal_cor, tgt_mat_xyz_cor, tgt_mat_normal_cor);
		demean_transform = incre_transform * demean_transform;

		// 5.5. pad n*3 matrix to n*4 and apply transform
		applyTransform(src_mat_xyz, src_mat_xyz, incre_transform);
		applyTransform(src_mat_normal, src_mat_normal, incre_transform);
		pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloud_pn_med_demean, *cloud_pn_med_demean, incre_transform);

		//! visualization for test!
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
		//	red_handler(cloud_pn_src_demean, 200, 20, 20), green_handler(cloud_pn_tgt_demean, 20, 200, 20), blue_handler(cloud_pn_med_demean, 20, 20, 200);
		//pcl::visualization::PCLVisualizer viewer("Viewer#1");
		//viewer.setBackgroundColor(255, 255, 255);
		//viewer.addPointCloud(cloud_pn_tgt_demean, green_handler, "cloud2");
		//viewer.addPointCloud(cloud_pn_med_demean, blue_handler, "cloud3");
		//while (!viewer.wasStopped()) viewer.spinOnce();

	}
	#pragma endregion 5. iterates until convergence / max


	// print iters result
	if (count > this->maxCounts) ;
	else if (iters == this->maxIters)
		cout << "[*] WARNING: Max iterations reached!" << endl;
	else
		cout << "[*] SUCCESS: Difference reaches below threshold!" << endl;
	cout << "[*] [ Iters#: " << iters << " - diff: " << diff / cloud_size * 1000 << "‰ ]" << endl;


	// 6. find un-demean transform
	// un-demean-transform = trans(tgt_mean) * demean-transform * trans(-src_mean)
	Eigen::Affine3f undemean_transform = Eigen::Translation3f(tgt_mean) * demean_transform * Eigen::Translation3f(-src_mean) * Eigen::Affine3f::Identity();
	//x undemean_transform.translate(-src_mean); 
	//x undemean_transform = demean_transform * undemean_transform; 
	//x undemean_transform.translate(tgt_mean);			// 这里虽然放在rotate的后面，但实际上和放在rotate前是一样的

	this->guess_transform = undemean_transform;

	return this->guess_transform;
}

// ---------------------

void MyICP::SetP2pParams(int maxIters)
{
	this->maxIters = maxIters;
}

Eigen::Affine3f MyICP::RegisterP2P()
{
	// icp
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(this->cloud_src);
	icp.setInputTarget(this->cloud_tgt);
	icp.setMaximumIterations(this->maxIters);
	//icp.setEuclideanFitnessEpsilon(0.0001);
	icp.align(*(this->cloud_apply));
	this->guess_transform = Eigen::Affine3f(icp.getFinalTransformation());

	return this->guess_transform;
}

void MyICP::SetUsePca(bool usePca)
{
	this->usePca = usePca;
}


// ---------------------


void MyICP::demean()
{
	//! resize, otherwise vector index out of range
	//x cloud_src_deman->width = cloud_src->width
	cloud_src_demean->resize(cloud_src->width);
	cloud_tgt_demean->resize(cloud_tgt->width);
	
	this->src_mean = Eigen::Vector3f::Zero(), this->tgt_mean = Eigen::Vector3f::Zero();

	for (int i = 0; i < cloud_src->width; i++)
	{
		this->src_mean[0] += cloud_src->points[i].x;
		this->src_mean[1] += cloud_src->points[i].y;
		this->src_mean[2] += cloud_src->points[i].z;
	}
	for (int i = 0; i < cloud_tgt->width; i++)
	{
		this->tgt_mean[0] += cloud_tgt->points[i].x;
		this->tgt_mean[1] += cloud_tgt->points[i].y;
		this->tgt_mean[2] += cloud_tgt->points[i].z;
	}
	this->src_mean /= cloud_src->width, this->tgt_mean /= cloud_tgt->width;

	for (int i = 0; i < cloud_src->width; i++)
	{
		cloud_src_demean->points[i].x = cloud_src->points[i].x - this->src_mean[0];
		cloud_src_demean->points[i].y = cloud_src->points[i].y - this->src_mean[1];
		cloud_src_demean->points[i].z = cloud_src->points[i].z - this->src_mean[2];
	}
	for (int i = 0; i < cloud_tgt->width; i++)
	{
		cloud_tgt_demean->points[i].y = cloud_tgt->points[i].y - this->tgt_mean[1];
		cloud_tgt_demean->points[i].z = cloud_tgt->points[i].z - this->tgt_mean[2];
		cloud_tgt_demean->points[i].x = cloud_tgt->points[i].x - this->tgt_mean[0];
	}
}

void MyICP::estimateNormals()
{
	pcl::PointCloud<pcl::Normal> normals_src, normals_tgt;

	// normal estimator
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	ne.setSearchMethod(tree);
	ne.setKSearch(10);
	//x ne.setRadiusSearch(0.5);		// 0.5 is too narrow!
	
	// estimate
	ne.setInputCloud(cloud_src_demean);
	ne.compute(normals_src);
	ne.setInputCloud(cloud_tgt_demean);
	ne.compute(normals_tgt);

	// concatenate
	pcl::concatenateFields(*cloud_src_demean, normals_src, *cloud_pn_src_demean);
	pcl::concatenateFields(*cloud_tgt_demean, normals_tgt, *cloud_pn_tgt_demean);
	*cloud_pn_med_demean = *cloud_pn_src_demean;
}

inline void MyICP::initCorrespondenceEstimation()
{
	this->ce = pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr(new pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal>);
	this->src_tree = pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>);
	this->tgt_tree = pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>);
	this->ce->setSearchMethodSource(this->src_tree), ce->setSearchMethodTarget(this->tgt_tree);

	ce->setInputSource(cloud_pn_med_demean), ce->setInputTarget(cloud_pn_tgt_demean);
	ce->determineReciprocalCorrespondences(correspondences);		// this will automatically reject the too-far-away-point-pairs
	//ce->determineCorrespondences(correspondences);

	pasteWithCorrespondence(correspondences, src_mat_xyz, src_mat_xyz_cor, tgt_mat_xyz, tgt_mat_xyz_cor);
}

Eigen::Affine3f MyICP::getInitTransform()
{
	Eigen::Affine3f demean_transform = Eigen::Affine3f::Identity();

	int max_cor_num = correspondences.size();
	cout << "Init correspondences points: " << max_cor_num << endl;

	//// visualization for test!
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
	//	red_handler(cloud_pn_src_demean, 200, 20, 20), green_handler(cloud_pn_tgt_demean, 20, 200, 20), blue_handler(cloud_pn_med_demean, 20, 20, 200);
	//pcl::visualization::PCLVisualizer viewer("Viewer#1");
	//viewer.setBackgroundColor(255, 255, 255);
	//viewer.addPointCloud(cloud_pn_tgt_demean, green_handler, "cloud2");
	//viewer.addPointCloud(cloud_pn_med_demean, blue_handler, "cloud3");
	//while (!viewer.wasStopped()) viewer.spinOnce();

	// initialize transform with PCA
	if (this->usePca)
	{
		std::vector<Eigen::Vector4f> src, tgt;
		get3Dpca(this->src_mat_xyz, src), get3Dpca(this->tgt_mat_xyz, tgt);

		// todo compress codes that are same with random guess #1
		bool isUpdated = false;
		float min_diff = evalDiff(src_mat_xyz_cor, tgt_mat_xyz_cor);
		Eigen::MatrixXf src_mat_xyz_guess = src_mat_xyz;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_med_demean_guess(new pcl::PointCloud<pcl::PointNormal>);

		for (int i = 0; i < 4; i++)
		{
			Eigen::Affine3f guess_demean_transform = getRotateMatrix(src, tgt, i / 2 >= 1, i % 2 / 1 >= 1);
			applyTransform(src_mat_xyz, src_mat_xyz_guess, guess_demean_transform);

			pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloud_pn_med_demean, *cloud_pn_med_demean_guess, guess_demean_transform);
			findCorrespondences(this->ce, cloud_pn_med_demean_guess, this->cloud_pn_tgt_demean, this->correspondences);

			Eigen::MatrixXf src_mat_xyz_cor_guess, tgt_mat_xyz_cor_guess;
			pasteWithCorrespondence(correspondences, src_mat_xyz_guess, src_mat_xyz_cor_guess, tgt_mat_xyz, tgt_mat_xyz_cor_guess);	//! 这里第2个参数要带 _guess 的！

			//! visualization for test!
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
				red_handler(cloud_pn_src_demean, 200, 20, 20), green_handler(cloud_pn_tgt_demean, 20, 200, 20), blue_handler(cloud_pn_med_demean_guess, 20, 20, 200);
			pcl::visualization::PCLVisualizer viewer("Viewer#1");
			viewer.setBackgroundColor(255, 255, 255);
			viewer.addPointCloud(cloud_pn_tgt_demean, green_handler, "cloud2");
			viewer.addPointCloud(cloud_pn_med_demean_guess, blue_handler, "cloud3");
			while (!viewer.wasStopped()) viewer.spinOnce();

			float new_diff = evalDiff(src_mat_xyz_cor_guess, tgt_mat_xyz_cor_guess);
			int new_cor_num = correspondences.size();

			if (new_cor_num > max_cor_num && new_diff < min_diff)
			{
				isUpdated = true;
				min_diff = new_diff;
				max_cor_num = new_cor_num;
				demean_transform = guess_demean_transform;
			}
		}
		
		if (isUpdated)
		{
			pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloud_pn_med_demean, *cloud_pn_med_demean, demean_transform);
			applyTransform(this->src_mat_xyz, this->src_mat_xyz, demean_transform);
			applyTransform(this->src_mat_normal, this->src_mat_normal, demean_transform);
		}
	}

	// initialize transform with random guesses
	if (this->guess_times > 0)
	{
		// todo compress codes that are same with pca #2
		bool isUpdated = false;
		float min_diff = evalDiff(src_mat_xyz_cor, tgt_mat_xyz_cor);
		Eigen::MatrixXf src_mat_xyz_guess = src_mat_xyz;
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_med_demean_guess(new pcl::PointCloud<pcl::PointNormal>);

		cout << "#  |  minimum diff |   new diff   |  correspondence pts   |  updated" << endl;

		for (int i = 0; i < this->guess_times; i++)
		{
			Eigen::Affine3f guess_demean_transform = getRandomRotate();
			applyTransform(src_mat_xyz, src_mat_xyz_guess, guess_demean_transform);

			pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloud_pn_med_demean, *cloud_pn_med_demean_guess, guess_demean_transform);
			findCorrespondences(this->ce, cloud_pn_med_demean_guess, this->cloud_pn_tgt_demean, this->correspondences);

			Eigen::MatrixXf src_mat_xyz_cor_guess, tgt_mat_xyz_cor_guess;
			pasteWithCorrespondence(correspondences, src_mat_xyz_guess, src_mat_xyz_cor_guess, tgt_mat_xyz, tgt_mat_xyz_cor_guess);	//! 这里第2个参数要带 _guess 的！

			float new_diff = evalDiff(src_mat_xyz_cor_guess, tgt_mat_xyz_cor_guess);
			int new_cor_num = correspondences.size();
			cout << std::left << setw(3) << i+1 << "|  " 
				<< setw(9) << min_diff << "    |   " 
				<< setw(9) << new_diff << "  |         " 
				<< setw(6) << new_cor_num << "        |    ";
			if ((new_cor_num > max_cor_num && (new_diff - min_diff) / min_diff <= DIFF_TOLERANCE) ||			// cor↑,  diff~
				(new_cor_num / max_cor_num > 0.9 && (new_diff - min_diff) / min_diff <= -DIFF_TOLERANCE) ||		// cor~,  diff↓
				(new_cor_num / max_cor_num > 1.6 && (new_diff - min_diff) / min_diff <= 6 * DIFF_TOLERANCE))	// cor↑↑, diff↑	
			{
				isUpdated = true;
				min_diff = new_diff;
				max_cor_num = new_cor_num;
				demean_transform = guess_demean_transform;
				cout << "***" << endl;
			}
			else
				cout << endl;
			////! visualization for test!
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
			//	red_handler(cloud_pn_src_demean, 200, 20, 20), green_handler(cloud_pn_tgt_demean, 20, 200, 20), blue_handler(cloud_pn_med_demean_guess, 20, 20, 200);
			//pcl::visualization::PCLVisualizer viewer("Viewer#1");
			//viewer.setBackgroundColor(255, 255, 255);
			//viewer.addPointCloud(cloud_pn_tgt_demean, green_handler, "cloud2");
			//viewer.addPointCloud(cloud_pn_med_demean_guess, blue_handler, "cloud3");
			//while (!viewer.wasStopped()) viewer.spinOnce();

		}

		if (isUpdated)
		{
			pcl::transformPointCloudWithNormals<pcl::PointNormal>(*cloud_pn_med_demean, *cloud_pn_med_demean, demean_transform);
			applyTransform(this->src_mat_xyz, this->src_mat_xyz, demean_transform);
			applyTransform(this->src_mat_normal, this->src_mat_normal, demean_transform);
		}
	}

	return demean_transform;
}

