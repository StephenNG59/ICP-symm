#pragma once

#include "stdafx.h"

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
	void RegisterSymm();		// todo add params to specify iters & diff


private:
	int max_iters;
	float diff_threshold;

	pcl::PCLPointCloud2::Ptr pclcloud_src, pclcloud_tgt;
	pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_src, cloud_pn_tgt;

	void estimateNormals();
	//pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt;
	
	//float evalDiff(const pcl::PointCloud<PointT>::Ptr cloud_src, const pcl::PointCloud<PointT>::Ptr cloud_tgt);

	//void estimateTransformSymm(const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tgt, Eigen::Vector3f& axis, float& theta, Eigen::Vector3f& translate);
};
