#pragma once

#include "stdafx.h"

constexpr int DEFAULT_MAX_ITERS = 100;
constexpr float DEFAULT_DIFF_THRESH = 0.1f;

typedef pcl::PointXYZ PointT;

class MyICP
{
public:
	MyICP();
	~MyICP();

public:
	int LoadCloud(std::string src_path, std::string tgt_path);
	pcl::PointCloud<PointT>::Ptr GetSrcCloud();
	pcl::PointCloud<PointT>::Ptr GetTgtCloud();

	void RegisterP2P();
	Eigen::Affine3f RegisterSymm();		// todo add params to specify iters & diff


private:
	int max_iters;
	float diff_threshold;

	pcl::PCLPointCloud2::Ptr pclcloud_src, pclcloud_tgt;
	pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_src, cloud_pn_tgt, cloud_pn_med;

	void estimateNormals();
};
