#pragma once
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <liblas/liblas.hpp>
#include <fstream>  // std::ifstream
#include "PointXYZDouble.h"


delegate void ProgressCallback(double readPoints, double totalPoints);
typedef void(*UnmanagedProgressCallback)(double readPoints, double totalPoints);

class LASFileReader
{
private:
	pcl::PointCloud<PointXYZDouble>::Ptr loaded_point_cloud;

public:
	LASFileReader();
	~LASFileReader();
	pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char *fileName);
	pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char *fileName, UnmanagedProgressCallback callback);

	pcl::PointCloud<PointXYZDouble>::Ptr LoadedData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr DecreaseDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	PointXYZDouble maxPoint(const char *fileName);
	PointXYZDouble minPoint(const char *fileName);
};

