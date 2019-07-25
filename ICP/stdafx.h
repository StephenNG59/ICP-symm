#pragma once

#include <iostream>
#include <iomanip>								// io ¶ÔÆë
#include <vector>
#include <string>
#include <assert.h>
#include <time.h>
#include <sstream>

#include <pcl\io\pcd_io.h>						// pcd files io
#include <pcl\io\obj_io.h>						// obj files io
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl/visualization/pcl_visualizer.h>	// pcl visualizer
#include <pcl\PCLPointCloud2.h>					// pcl::PCLPointCloud2
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>				// normal estimation
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>				// iterative closest point

#include <boost/algorithm/string.hpp>			// split


#include <opencv2/opencv.hpp>
