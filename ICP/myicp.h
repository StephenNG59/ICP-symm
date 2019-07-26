#pragma once

#include "stdafx.h"

constexpr int DEFAULT_MAX_ITERS = 200;
constexpr float DEFAULT_DIFF_THRESH = 0.001f;
constexpr int COUNT_MAX = 40;				// 迭代不理想的次数，到达次数后自动退出
constexpr int DEFAULT_GUESS_TIMES = 0;
constexpr float DIFF_TOLERANCE = 0.08;		// 新的guess的diff必须比目前最好的guess的diff大不超过0.05
constexpr float MIN_COR_RATIO = 0.01;		// 带有rejection的correspondence最少要有原先的0.1的点
constexpr float MED_COR_RATIO = 0.6;		// 
//constexpr float TEMPERATURE_INIT = 100, ANNEAL_RATIO = 0.98, TEMPERATURE_MIN = 0;


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
	inline void SetGuessTimes(int t) { guess_times = t; cout << "set guess times to:" << t << endl; }


private:
	int max_iters;
	float diff_threshold;
	int guess_times;
	Eigen::Affine3f guess_transform;
	Eigen::Vector3f src_mean, tgt_mean;

	pcl::PCLPointCloud2::Ptr pclcloud_src, pclcloud_tgt;
	pcl::PointCloud<PointT>::Ptr cloud_src, cloud_tgt, cloud_src_demean, cloud_tgt_demean, cloud_apply;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn_src_demean, cloud_pn_tgt_demean, cloud_pn_med_demean;

	void demean();
	void estimateNormals();
};
