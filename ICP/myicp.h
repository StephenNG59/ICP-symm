#pragma once

#include "stdafx.h"

constexpr int DEFAULT_MAX_ITERS = 200;
constexpr float DEFAULT_DIFF_THRESH = 0.001f;
constexpr int COUNT_MAX = 40;

typedef pcl::PointXYZ PointT;

class MyICP
{
public:
	MyICP();
	~MyICP();

public:
	int LoadCloudPcd(std::string src_path, std::string tgt_path);
	int LoadCloudObj(std::string src_path, std::string tgt_path);
	pcl::PointCloud<PointT>::Ptr GetSrcCloud();
	pcl::PointCloud<PointT>::Ptr GetTgtCloud();

	void RegisterP2P();
	Eigen::Affine3f RegisterSymm(float diff_threshold = DEFAULT_DIFF_THRESH, int max_iters = DEFAULT_MAX_ITERS);
	void Visualize(bool showOnce = false);


private:
	int max_iters;
	float diff_threshold;
	Eigen::Affine3f guess_transform;
	Eigen::Vector3f src_mean, tgt_mean;

	pcl::PCLPointCloud2::Ptr pclcloud_src, pclcloud_tgt;
	pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt, cloud_src_demean, cloud_tgt_demean, cloud_apply;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_src_demean, cloud_pn_tgt_demean, cloud_pn_med_demean;

	void demean();
	void estimateNormals();
};
