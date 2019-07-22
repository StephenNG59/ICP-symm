#include "stdafx.h"
#include "func.h"


void pasteInMatrix(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::MatrixXf& mat_XYZ, Eigen::MatrixXf& mat_normal)
{
	int npts = cloud->points.size();
	mat_XYZ = Eigen::MatrixXf(npts, 3), mat_normal = Eigen::MatrixXf(npts, 3);

	for (size_t i = 0; i < npts; i++)
	{
		mat_XYZ.row(i) = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		mat_normal.row(i) = Eigen::Vector3f(cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z);
	}
}


// calculate diff of two point sets in matrix [m * n]. m is #points, n is #dimensions. src and tgt is exactly corresponded.
float evalDiff(const Eigen::MatrixXf& src, const Eigen::MatrixXf& tgt)
{
	assert(src.rows() == tgt.rows() && src.cols() == tgt.cols());

	float diff = 0.f;
	size_t npts = src.rows();

	for (size_t i = 0; i < npts; i++)
	{
		diff += (src.row(i) - tgt.row(i)).norm();
	}

	return diff;
}


// calculate diff of two point sets in matrix [m * n]. m is #points, n is #dimensions. src and tgt is corresponded based on indices.
float evalDiff(const Eigen::MatrixXf& src, const Eigen::MatrixXf& tgt, const std::vector<int>& indices)
{
	// todo complete this
	return 0.f;
}


//x N��c���Ǽ������ˡ���
void calculateMatrixNotation(const Eigen::MatrixXf& src_mat_xyz, const Eigen::MatrixXf& src_mat_normal, const Eigen::MatrixXf& tgt_mat_xyz, const Eigen::MatrixXf& tgt_mat_normal, Eigen::MatrixXf& M, Eigen::MatrixXf& N, Eigen::VectorXf& c)
{
	int npts = src_mat_xyz.rows();
	M = Eigen::MatrixXf(npts, 3), N = Eigen::MatrixXf(npts, 3), c = Eigen::VectorXf(npts);

	Eigen::Vector3f p, q, n;			// cross can only apply on 3d vectors
	for (size_t i = 0; i < npts; i++)
	{
		p = src_mat_xyz.row(i), q = tgt_mat_xyz.row(i), n = src_mat_normal.row(i) + tgt_mat_normal.row(i);

		// m_i = (src_demean_i + tgt_demean_i) �� (src_normal_i + tgt_normal_i)
		M.row(i) = (p + q).cross(n);
		// n_i = (src_normal_i + tgt_normal_i)
		N.row(i) = n;
		// c_i = (src_demean_i - tgt_demean_i) �� (src_normal_i + tgt_normal_i)
		c(i) = (p - q).dot(n);
	}
}


// solve object function: argmin(x) F(x) = ||Ax - b||_2
Eigen::VectorXf solveLLS(const Eigen::MatrixXf& A, const Eigen::VectorXf& b)
{
	// SVD of A: U * �� * V^T ([m*n] * [n*n] * [n*n])
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);		//? thinU: m * n, fullV: n * n?

	// solution vector: x = V * ��^-1 * U^T * b
	Eigen::VectorXf x = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose() * b;

	return x;
}


Eigen::Affine3f estimateTransformSymm(const Eigen::MatrixXf& src_mat_xyz, const Eigen::MatrixXf& src_mat_normal, const Eigen::MatrixXf& tgt_mat_xyz, const Eigen::MatrixXf& tgt_mat_normal)
{
	// 1. get matrix notation of M & N, get vector notation of c
	// ---------------------------------------------------------
	Eigen::MatrixXf M, N;
	Eigen::VectorXf c;
	calculateMatrixNotation(src_mat_xyz, src_mat_normal, tgt_mat_xyz, tgt_mat_normal, M, N, c);


	// 2. solve LLS by SVD for a_ and t_ 
	// ---------------------------------
	//	�� = ||c + M * a_ + N * t_||^2, with a_ = aixs * tan(��), t_ = translate / cos(��)
	Eigen::Vector3f src_mean = src_mat_xyz.colwise().mean(), tgt_mean = tgt_mat_xyz.colwise().mean();
	Eigen::Vector3f t_ = tgt_mean - src_mean, a_;		//? now initialize t_ with diff of mean and use this t_ to calc a_. (initialize t_ = 0 will result in chaos)
	a_ = solveLLS(M, -(N * t_ + c)), t_ = solveLLS(N, -(M * a_ + c));


	// 3. calculate transform matrix based on a_ and t_
	// ------------------------------------------------
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	// �� = arctan(||a_||)
	float theta = atan(a_.norm());
	//! transform = rot(��, a_) * trans(t_ * ��) * rot(��, a_)
	//x transform = trans(tgt_mean) * rot(��, a_) * trans(t_ * ��) * rot(��, a_) * trans(-src_mean)
	//x transform.translate(-src_mean);
	transform.rotate(Eigen::AngleAxisf(theta, a_ / a_.norm()));
	transform.translate(t_ * cos(theta));
	transform.rotate(Eigen::AngleAxisf(theta, a_ / a_.norm()));
	//x transform.translate(tgt_mean);

	return transform;
}


void applyTransform(Eigen::MatrixXf& src, Eigen::MatrixXf& tgt, Eigen::Affine3f& transform)
{
	// only takes n*3 matrix...
	assert(src.cols() == 3 && src.rows() == tgt.rows());

	// pad n*3 to n*4
	int npts = src.rows();
	Eigen::MatrixXf med(npts, 4);
	for (size_t i = 0; i < npts; i++)
	{
		med.row(i) << src.row(i), 1.0f;
	}

	// apply transform
	med = (transform.matrix() * med.transpose()).transpose();		//! should use Eigen::Affine3f.matrix()
	//x tgt = med.head<3>();		// error: you tried calling a vector method on a matrix
	tgt = med.leftCols(3);
}

void pasteWithCorrespondence(pcl::Correspondences& correspondences, Eigen::MatrixXf& src_mat, Eigen::MatrixXf& src_mat_cor, Eigen::MatrixXf& tgt_mat, Eigen::MatrixXf& tgt_mat_cor)
{
	int size = correspondences.size();
	src_mat_cor = Eigen::MatrixXf(size, 3), tgt_mat_cor = Eigen::MatrixXf(size, 3);

	for (size_t i = 0; i < size; i++)
	{
		src_mat_cor.row(i) = src_mat.row(correspondences[i].index_query);
		tgt_mat_cor.row(i) = tgt_mat.row(correspondences[i].index_match);
	}
}

void findCorrespondences(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt, pcl::Correspondences& correspondences)
{
	pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal> ce;
	ce.setInputSource(cloud_src), ce.setInputTarget(cloud_tgt);
	ce.determineReciprocalCorrespondences(correspondences);		//! this will automatically reject the too-far-away-point-pairs
																//x ce.determineCorrespondences(correspondences);
}

