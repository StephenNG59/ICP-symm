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

//x N和c忘记加引用了……
void calculateMatrixNotation(const Eigen::MatrixXf& src_mat_xyz, const Eigen::MatrixXf& src_mat_normal, const Eigen::MatrixXf& tgt_mat_xyz, const Eigen::MatrixXf& tgt_mat_normal, Eigen::MatrixXf& M, Eigen::MatrixXf& N, Eigen::VectorXf& c)
{
	int npts = src_mat_xyz.rows();
	M = Eigen::MatrixXf(npts, 3), N = Eigen::MatrixXf(npts, 3), c = Eigen::VectorXf(npts);

	Eigen::Vector3f p, q, n;			// cross can only apply on 3d vectors
	for (size_t i = 0; i < npts; i++)
	{
		p = src_mat_xyz.row(i), q = tgt_mat_xyz.row(i), n = src_mat_normal.row(i) + tgt_mat_normal.row(i);

		// m_i = (src_demean_i + tgt_demean_i) × (src_normal_i + tgt_normal_i)
		M.row(i) = (p + q).cross(n);
		// n_i = (src_normal_i + tgt_normal_i)
		N.row(i) = n;
		// c_i = (src_demean_i - tgt_demean_i) · (src_normal_i + tgt_normal_i)
		c(i) = (p - q).dot(n);
	}
}


// solve object function: argmin(x) F(x) = ||Ax - b||_2
Eigen::VectorXf solveLLS(const Eigen::MatrixXf& A, const Eigen::VectorXf& b)
{
	// SVD of A: U * Σ * V^T ([m*n] * [n*n] * [n*n])
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);		//? thinU: m * n, fullV: n * n?

	// solution vector: x = V * Σ^-1 * U^T * b
	Eigen::VectorXf x = svd.matrixV() * svd.singularValues().asDiagonal().inverse() * svd.matrixU().transpose() * b;

	return x;
}


Eigen::Affine3f estimateTransformSymm(const Eigen::MatrixXf& src_mat_xyz, const Eigen::MatrixXf& src_mat_normal, const Eigen::MatrixXf& tgt_mat_xyz, const Eigen::MatrixXf& tgt_mat_normal)
{
	// get matrix notation of M & N, get vector notation of c
	Eigen::MatrixXf M, N;
	Eigen::VectorXf c;
	calculateMatrixNotation(src_mat_xyz, src_mat_normal, tgt_mat_xyz, tgt_mat_normal, M, N, c);

	// solve LLS by SVD for a_ and t_ 
	//	ε = ||c + M * a_ + N * t_||^2, with a_ = aixs * tan(θ), t_ = translate / cos(θ)
	Eigen::Vector3f src_mean = src_mat_xyz.colwise().mean(), tgt_mean = tgt_mat_xyz.colwise().mean();
	Eigen::Vector3f a_, t_ = tgt_mean - src_mean;		//? now initialize t_ with translate and use it to calc a_. reasonable?
	a_ = solveLLS(M, -(N * t_ + c));					//! no normals, so N is [0*0]
	t_ = solveLLS(N, -(M * a_ + c));

	// calculate transform matrix based on a_ and t_
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	// θ = arctan(||a_||)
	float theta = atan(a_.norm());
	// transform = trans(tgt_mean) * rot(θ, a_) * trans(t_ * θ) * rot(θ, a_) * trans(-src_mean)
	transform.translate(-src_mean);
	transform.rotate(Eigen::AngleAxisf(theta, a_ / a_.norm()));
	transform.translate(t_ * cos(theta));
	transform.rotate(Eigen::AngleAxisf(theta, a_ / a_.norm()));
	transform.translate(tgt_mean);

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


/*void estimateTransformSymm(const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tgt, Eigen::Vector3f& axis, float& theta, Eigen::Vector3f& translate)
{
	int npts = cloud_src->points.size();

	// matrix to store XYZ & normals
	Eigen::MatrixXf mat_src(npts, 3), mat_src_normal(npts, 3), mat_tgt(npts, 3), mat_tgt_normal(npts, 3);
	cout << mat_src.rows() << mat_src.cols() << endl;

	// copy meanwhile demean, and get the [translate]
	translate = pasteInMatrix(cloud_src, cloud_tgt, mat_src, mat_src_normal, mat_tgt, mat_tgt_normal);

	// calculate matrix notation M & N, and vector c
	Eigen::MatrixXf M(npts, 3), N(npts, 3);
	Eigen::VectorXf c(npts);
	calculateMatrixNotation(mat_src, mat_src_normal, mat_tgt, mat_tgt_normal, M, N, c);

	// solve LLS by SVD for a_ and t_ 
	//	ε = ||c + M * a_ + N * t_||_2
	//	其中a_ = a * tan(θ), t_ = t / cos(θ)
	Eigen::Vector3f a_, t_;
	solveLLS(M, -(N * (translate / cos(theta)) + c), a_);		// TODO: divide by 0
	solveLLS(N, -(M * (axis * tan(theta)) + c), t_);

	// calculate [axis] & [theta] by a_ & t_
	cout << "translate / t_:" << translate[0] / t_[0] << translate[1] / t_[1] << translate[2] / t_[2] << endl;
	theta = acos(translate.norm() / t_.norm());		// TODO: if translate == 0, theta -> 90°
	axis = a_ * tan(theta);
}*/
