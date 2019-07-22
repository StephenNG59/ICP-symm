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

	//estimateNormals();

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
	Eigen::MatrixXf matrix_src, matrix_tgt;
	pcl::getPointCloudAsEigen(*pclcloud_src, matrix_src);

	// transform
	Eigen::Affine3f guess = Eigen::Affine3f::Identity();
	//guess.translation() << 0.0, 0.0, 0.0;
	//guess.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));

	cout << "guess matrix:\n" << guess.matrix() << endl;

	pcl::transformPointCloud(*cloud_src, *cloud_src, guess);
	//matrix_tgt = transform * matrix_src;

	
}

/*void MyICP::RegisterSymm()
{
	assert(cloud_src && cloud_tgt);

	// median pointcloud
	pcl::PointCloud<PointT>::Ptr cloud_med(new pcl::PointCloud<PointT>);
	*cloud_med = *cloud_src;
	
	// initialize axis->a=UnitZ, translate->t=0, angle->theta=0
	Eigen::Vector3f a_ = Eigen::Vector3f::Zero(), t_ = Eigen::Vector3f::Zero();
	float theta = 0.f;
	Eigen::Affine3f guess = Eigen::Affine3f::Identity();
	//guess.translation() << 2.5f, 0.0f, 0.0f;
	guess.translation() = t;
	guess.rotate(Eigen::AngleAxisf(theta, a));

	// iterations
	float diff = 0.f;
	int iters = 0;
	while ((diff = evalDiff(cloud_med, cloud_tgt)) > 10.0f && iters <= 10)
	{
		iters++;
		cout << "iters#" << iters << " - diff:" << diff << endl;

		// find correspondence
		//indices = ...
		
		// estimate transformation based on correspondence
		estimateTransformSymm(cloud_med, cloud_tgt, a, theta, t);

		// apply transform to median pointcloud
		guess.translate(t);
		guess.rotate(Eigen::AngleAxisf(theta, a));
		pcl::transformPointCloud(*cloud_med, *cloud_med, guess);
	}

}*/


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

/*float MyICP::evalDiff(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt)
{
	float diff = 0.f;

	for (size_t i = 0; i < cloud_src->size(); i++)
	{
		float d = pow(cloud_src->points[i].x - cloud_tgt->points[i].x, 2)
			+ pow(cloud_src->points[i].y - cloud_tgt->points[i].y, 2)
			+ pow(cloud_src->points[i].z - cloud_tgt->points[i].z, 2);
		diff += sqrt(d);
	}

	return diff;
}*/





/*void MyICP::pasteInMatrix(const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tgt, Eigen::MatrixXf& mat_src, Eigen::MatrixXf& mat_src_normal, Eigen::MatrixXf& mat_tgt, Eigen::MatrixXf& mat_tgt_normal)
{
	// TODO: if matrix size not matched

	int npts = cloud_src->points.size();

	mat_src = Eigen::MatrixXf(npts, 3);

	// copy pointcloud data into eigen matrix
	for (size_t i = 0; i < npts; i++)
	{
		mat_src(i, 0) = cloud_src->points[i].x,
			mat_src(i, 1) = cloud_src->points[i].y,
			mat_src(i, 2) = cloud_src->points[i].z;
		mat_src_normal(i, 0) = cloud_src->points[i].normal_x,
			mat_src_normal(i, 1) = cloud_src->points[i].normal_y,
			mat_src_normal(i, 2) = cloud_src->points[i].normal_z;

		int index = i;	//! should be indices[i] or [modify cloud_tgt in advance to match cloud_src]
		mat_tgt(i, 0) = cloud_tgt->points[index].x,
			mat_tgt(i, 1) = cloud_tgt->points[index].y,
			mat_tgt(i, 2) = cloud_tgt->points[index].z;
		mat_tgt_normal(i, 0) = cloud_tgt->points[index].normal_x,
			mat_tgt_normal(i, 1) = cloud_tgt->points[index].normal_y,
			mat_tgt_normal(i, 2) = cloud_tgt->points[index].normal_z;
	}

	// demean
	Eigen::Vector3f src_mean = mat_src.colwise().mean(), tgt_mean = mat_tgt.colwise().mean();
	mat_src = mat_src.rowwise() - src_mean.transpose(), mat_tgt = mat_tgt.rowwise() - tgt_mean.transpose();		//! default is column vector, need to transpose.

	// return the translation vector
	return (tgt_mean - src_mean);
}*/
