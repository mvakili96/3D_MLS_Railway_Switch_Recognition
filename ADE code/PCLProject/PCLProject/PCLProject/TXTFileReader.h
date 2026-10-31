#pragma once
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <liblas/liblas.hpp>
#include <fstream>  // std::ifstream
#include "PointXYZDouble.h"
#include "LASFileReader.h"
#include "LIDARPT.h"

class TXTFileReader
{
private:
	pcl::PointCloud<PointXYZDouble>::Ptr loaded_point_cloud;

public:
	TXTFileReader();
	~TXTFileReader();

	PointXYZDouble minPt;
	PointXYZDouble maxPt;

	pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char *fileName);
	pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char *fileName, UnmanagedProgressCallback callback);
	pcl::PointCloud<PointXYZDouble>::Ptr ReadDataCSV(const char* fileName, UnmanagedProgressCallback callback);
	pcl::PointCloud<PointXYZDouble>::Ptr LoadedData();
	void saveRailVector(const char *filename, vector<POINT3D> vtx);
	void saveRailVector(const char* filename, vector<POINT3D> vtx, vector<POINT3D> vtl, vector<POINT3D> vtr);
	void saveRailVectorWithIndex(const char* filename, vector<POINT3D> vtx, vector<POINT3D> vtl, vector<POINT3D> vtr);
};
