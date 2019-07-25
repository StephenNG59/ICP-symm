#include "stdafx.h"

#include "myicp.h"
#include "func.h"

MyICP::MyICP() : max_iters(DEFAULT_MAX_ITERS), diff_threshold(DEFAULT_DIFF_THRESH)
{
	pclcloud_src = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
	pclcloud_tgt = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
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

int MyICP::LoadCloudPcd(std::string src_path, std::string tgt_path)
{
	pcl::PCDReader reader;
	reader.read(src_path, *pclcloud_src);
	pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
	reader.read(tgt_path, *pclcloud_tgt);
	pcl::fromPCLPointCloud2(*pclcloud_tgt, *cloud_tgt);

	cout << "[-] source pts: " << cloud_src->width << " || target pts: " << cloud_tgt->width << endl;

	return 0;
}

int MyICP::LoadCloudObj(std::string src_path, std::string tgt_path)
{
	pcl::OBJReader reader;
	reader.read(src_path, *pclcloud_src);
	pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
	reader.read(tgt_path, *pclcloud_tgt);
	pcl::fromPCLPointCloud2(*pclcloud_tgt, *cloud_tgt);

	cout << "[-] source pts: " << cloud_src->width << " || target pts: " << cloud_tgt->width << endl;

	return 0;
}

pcl::PointCloud<PointT>::Ptr MyICP::GetSrcCloud()
{
	return this->cloud_src;
}

pcl::PointCloud<PointT>::Ptr MyICP::GetTgtCloud()
{
	return this->cloud_tgt;
}

void MyICP::RegisterP2P()
{

}

Eigen::Affine3f MyICP::RegisterSymm(float diff_threshold/* = DEFAULT_DIFF_THRESH*/, int max_iters/* = DEFAULT_MAX_ITERS*/)
{
	assert(cloud_src && cloud_tgt);
	float cloud_size = cloud_src->getMatrixXfMap().maxCoeff() - cloud_src->getMatrixXfMap().minCoeff();
	this->diff_threshold = diff_threshold * cloud_size, this->max_iters = max_iters;

	// 0. demean (XYZ -> demean)
	demean();


	// 1. estimate normals (use demean XYZ, concatenate with normal)
	estimateNormals();


	// 2. copy demean src & tgt cloud into eigen matrix
	Eigen::MatrixXf src_mat_xyz, src_mat_normal, tgt_mat_xyz, tgt_mat_normal;
	pasteInMatrix(cloud_pn_src_demean, src_mat_xyz, src_mat_normal);
	pasteInMatrix(cloud_pn_tgt_demean, tgt_mat_xyz, tgt_mat_normal);


	// 3. initialize transform = Identity
	Eigen::Affine3f demean_transform = Eigen::Affine3f::Identity();


	// 4. initialize correspondences estimation
	pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr ce(new pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr src_tree(new pcl::search::KdTree<pcl::PointNormal>), tgt_tree(new pcl::search::KdTree<pcl::PointNormal>);
	ce->setSearchMethodSource(src_tree), ce->setSearchMethodTarget(tgt_tree);


	// 5. iterates until convergence / max
	int iters = 0, count = 0;
	//float diff = evalDiff(src_mat_xyz, tgt_mat_xyz);
	float diff = diff_threshold + 1;
	while (diff > this->diff_threshold && iters < this->max_iters)
	{

		// 5.1. find correspondence
		pcl::Correspondences correspondences;
		ce->setInputSource(cloud_pn_med_demean), ce->setInputTarget(cloud_pn_tgt_demean);
		ce->determineReciprocalCorrespondences(correspondences);		//! this will automatically reject the too-far-away-point-pairs
		/*findCorrespondences(cloud_pn_med_demean, cloud_pn_tgt_demean, correspondences, true);
		if (correspondences.size() == 0)
		{
			findCorrespondences(cloud_pn_med_demean, cloud_pn_tgt_demean, correspondences, false);
			cout << "[*] WARNING: point sets may not converge!" << endl;
		}*/
		
		// 5.2. paste into new matrix based on correspondences
		Eigen::MatrixXf src_mat_xyz_cor, src_mat_normal_cor, tgt_mat_xyz_cor, tgt_mat_normal_cor;
		pasteWithCorrespondence(correspondences, src_mat_xyz, src_mat_xyz_cor, tgt_mat_xyz, tgt_mat_xyz_cor);
		pasteWithCorrespondence(correspondences, src_mat_normal, src_mat_normal_cor, tgt_mat_normal, tgt_mat_normal_cor);

		// 5.3. evaluate diff
		//! bring it to here
		float next_diff = evalDiff(src_mat_xyz_cor, tgt_mat_xyz_cor);
		if (next_diff < diff_threshold)
		{
			diff = next_diff;
			break;
		}
		if ((diff - next_diff) / diff < 0.001)
			if (count++ > COUNT_MAX)
			{
				cerr << "[*] WARNING: seems like this cannot progress..." << endl;
				break;
			}
		diff = next_diff;
		//! bring it to here
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
		//x cout << "  current transform" << endl << transform.matrix() << endl;


		//! visualization for test!
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>
		//	red_handler(cloud_pn_src_demean, 200, 20, 20), green_handler(cloud_pn_tgt_demean, 20, 200, 20), blue_handler(cloud_pn_med_demean, 20, 20, 200);
		//pcl::visualization::PCLVisualizer viewer("Viewer#1");
		//viewer.setBackgroundColor(255, 255, 255);
		////viewer.addPointCloud(cloud_pn_src_demean, red_handler, "cloud1");
		//viewer.addPointCloud(cloud_pn_tgt_demean, green_handler, "cloud2");
		//viewer.addPointCloud(cloud_pn_med_demean, blue_handler, "cloud3");
		//while (!viewer.wasStopped()) viewer.spinOnce();

	}

	// print iters result
	if (count > COUNT_MAX) ;
	//else if (diff > this->diff_threshold)
	else if (iters == this->max_iters)
		cout << "[*] WARNING: Max iterations reached!" << endl;
	else
		cout << "[*] SUCCESS: Difference reaches below threshold!" << endl;
	cout << "[*] [ Iters#: " << iters << " - diff: " << diff / cloud_size * 1000 << "‰ ]" << endl;


	// 5. find un-demean transform
	//! un-demean-transform = trans(tgt_mean) * demean-transform * trans(-src_mean)
	Eigen::Affine3f undemean_transform = Eigen::Translation3f(tgt_mean) * demean_transform * Eigen::Translation3f(-src_mean) * Eigen::Affine3f::Identity();
	//x undemean_transform.translate(-src_mean); 
	//x undemean_transform = demean_transform * undemean_transform; 
	//x undemean_transform.translate(tgt_mean);			// 这里虽然放在rotate的后面，但实际上和放在rotate前是一样的


	// 6. print results
	guess_transform = undemean_transform;
	std::cout << "Guess transform:" << endl
		<< guess_transform.matrix() << endl;


	return guess_transform;
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
	src_mean /= cloud_src->width, tgt_mean /= cloud_tgt->width;

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
