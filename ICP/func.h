#pragma once

#include "stdafx.h"

void pasteInMatrix(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, Eigen::MatrixXf& mat_xyz, Eigen::MatrixXf& mat_normal);

// calculate diff of two point sets in matrix [m * n]. m is #points, n is #dimensions. src and tgt is exactly corresponded.
float evalDiff(const Eigen::MatrixXf& src, const Eigen::MatrixXf& tgt);

// calculate diff of two point sets in matrix [m * n]. m is #points, n is #dimensions. src and tgt is corresponded based on indices.
float evalDiff(const Eigen::MatrixXf& src, const Eigen::MatrixXf& tgt, const std::vector<int>& indices);

void calculateMatrixNotation(const Eigen::MatrixXf& mat_src, const Eigen::MatrixXf& mat_src_normal, const Eigen::MatrixXf& mat_tgt, const Eigen::MatrixXf& mat_tgt_normal, Eigen::MatrixXf& M, Eigen::MatrixXf& N, Eigen::VectorXf& c);

// solve object function: argmin(x) F(x) = ||Ax - b||_2
Eigen::VectorXf solveLLS(const Eigen::MatrixXf& A, const Eigen::VectorXf& b);

//void estimateTransformSymm(const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tgt, Eigen::Vector3f& axis, float& theta, Eigen::Vector3f& translate);

Eigen::Affine3f estimateTransformSymm(
	const Eigen::MatrixXf& src_mat_xyz,
	const Eigen::MatrixXf& src_mat_normal,
	const Eigen::MatrixXf& tgt_mat_xyz,
	const Eigen::MatrixXf& tgt_mat_normal
);

void applyTransform(Eigen::MatrixXf& src, Eigen::MatrixXf& tgt, Eigen::Affine3f& transform);