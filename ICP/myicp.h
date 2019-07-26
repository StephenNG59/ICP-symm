#pragma once

#include "stdafx.h"

constexpr int DEFAULT_MAX_ITERS = 200;
constexpr float DEFAULT_DIFF_THRESH = 0.001f;
constexpr int COUNT_MAX = 40;				// 迭代不理想的次数，到达次数后自动退出
constexpr int DEFAULT_GUESS_TIMES = 0;
constexpr float DIFF_TOLERANCE = 0.08;		// 新的guess的diff必须比目前最好的guess的diff大不超过0.05
constexpr float MIN_COR_RATIO = 0.01;		// 带有rejection的correspondence最少要有原先的0.1的点
constexpr float MED_COR_RATIO = 0.6;		// 
//constexpr float TEMPERATURE_INIT = 100, ANNEAL_RATIO = 0.98, TEMPERATURE_MIN = 10;


typedef pcl::PointXYZ PointT;

class MyICP
{
public:
	MyICP();
	~MyICP();

public:
	int LoadCloudFiles(std::string src_path, std::string tgt_path);
	void Visualize(bool showOnce = false);

	// registration
	void SetSymmParams(int maxIters, float diffThreshold, int guessTimes);
	Eigen::Affine3f RegisterSymm();
	void SetP2pParams(int maxIters);
	Eigen::Affine3f RegisterP2P();


private:
	int maxIters;
	float diffThreshold;
	int guess_times;

	Eigen::Affine3f guess_transform;

	Eigen::Vector3f src_mean, tgt_mean;
	Eigen::MatrixXf src_mat_xyz, src_mat_normal, tgt_mat_xyz, tgt_mat_normal;
	Eigen::MatrixXf src_mat_xyz_cor, src_mat_normal_cor, tgt_mat_xyz_cor, tgt_mat_normal_cor;

	pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt, cloud_src_demean, cloud_tgt_demean, cloud_apply;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_src_demean, cloud_pn_tgt_demean, cloud_pn_med_demean;

	pcl::Correspondences correspondences;
	pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr ce;
	pcl::search::KdTree<pcl::PointNormal>::Ptr src_tree, tgt_tree;

	void demean();
	void estimateNormals();
	void initCorrespondenceEstimation();

	Eigen::Affine3f getInitTransform();
};
