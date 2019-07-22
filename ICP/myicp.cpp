#include "stdafx.h"

#include "myicp.h"
#include "func.h"

MyICP::MyICP() : max_iters(10), diff_threshold(1.f)
{
	pclcloud_src = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
	pclcloud_tgt = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
	cloud_src = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_tgt = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	cloud_pn_src = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
	cloud_pn_tgt = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
}

MyICP::~MyICP()
{
}

int MyICP::LoadCloud(std::string src_path, std::string tgt_path)
{
	pcl::PCDReader reader;
	reader.read(src_path, *pclcloud_src);
	pcl::fromPCLPointCloud2(*pclcloud_src, *cloud_src);
	reader.read(tgt_path, *pclcloud_tgt);
	pcl::fromPCLPointCloud2(*pclcloud_tgt, *cloud_tgt);

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

void MyICP::RegisterSymm()
{
	assert(cloud_src && cloud_tgt);

	// 0. estimate normals
	estimateNormals();

	// 1. copy src and tgt cloud into eigen matrix
	// -------------------------------------------
	Eigen::MatrixXf src_mat_xyz, src_mat_normal, tgt_mat_xyz, tgt_mat_normal;
	pasteInMatrix(cloud_pn_src, src_mat_xyz, src_mat_normal);
	pasteInMatrix(cloud_pn_tgt, tgt_mat_xyz, tgt_mat_normal);

	// 2. initialize a_ = (0,0,0) & t_ = q_mean - p_mean & transform = Identity
	// ------------------------------------------------------------------------
	//x Eigen::Vector3f src_mean = src_mat_xyz.colwise().mean(), tgt_mean = tgt_mat_xyz.colwise().mean();
	//x Eigen::Vector3f a_ = Eigen::Vector3f::Zero(), t_ = tgt_mean - src_mean;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// 3. iterates until convergence / max
	// -----------------------------------
	int iters = 0;
	float diff = evalDiff(src_mat_xyz, tgt_mat_xyz);
	while (diff > diff_threshold && iters++ < max_iters)
	{
		cout << "iters#" << iters << endl
			<< "diff: " << diff << endl;

		// 3.1. find correspondence and sort tgt_mat_xyz and tgt_mat_normal by indices
		// todo indices = ...
		Eigen::MatrixXf tgt_mat_xyz_cor = tgt_mat_xyz, tgt_mat_normal_cor = tgt_mat_normal;
		// todo tgt_mat_xyz_cor = ...; tgt_mat_normal_cor = ...;

		// 3.2. estimate best transform from src to tgt_cor, apply it
		Eigen::Affine3f incre_transform = estimateTransformSymm(src_mat_xyz, src_mat_normal, tgt_mat_xyz_cor, tgt_mat_normal_cor);
		// pad n*3 matrix to n*4 and apply transform
		applyTransform(src_mat_xyz, src_mat_xyz, incre_transform);
		applyTransform(src_mat_normal, src_mat_normal, incre_transform);
		transform = incre_transform * transform;

		// 3.3. evaluate diff
		diff = evalDiff(src_mat_xyz, tgt_mat_xyz);
	}

	// 4. print results
	// ----------------
	std::cout << "Result transform:" << endl
		<< transform.matrix() << endl
		<< "  rotation:\n" << transform.rotation() << endl
		<< "  translation:\n" << transform.translation() << endl;
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
	ne.setInputCloud(cloud_src);
	ne.compute(normals_src);
	ne.setInputCloud(cloud_tgt);
	ne.compute(normals_tgt);

	// concatenate
	pcl::concatenateFields(*cloud_src, normals_src, *cloud_pn_src);
	pcl::concatenateFields(*cloud_tgt, normals_tgt, *cloud_pn_tgt);
}
