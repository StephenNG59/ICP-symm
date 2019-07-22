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


Eigen::Affine3f estimateTransformSymm(
	const Eigen::MatrixXf& src_mat_xyz,
	const Eigen::MatrixXf& src_mat_normal,
	const Eigen::MatrixXf& tgt_mat_xyz,
	const Eigen::MatrixXf& tgt_mat_normal
);


void applyTransform(Eigen::MatrixXf& src, Eigen::MatrixXf& tgt, Eigen::Affine3f& transform);


void pasteWithCorrespondence(pcl::Correspondences& correspondences, Eigen::MatrixXf& src_mat, Eigen::MatrixXf& src_mat_cor, Eigen::MatrixXf& tgt_mat, Eigen::MatrixXf& tgt_mat_cor);


void findCorrespondences(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt, pcl::Correspondences& correspondences);

