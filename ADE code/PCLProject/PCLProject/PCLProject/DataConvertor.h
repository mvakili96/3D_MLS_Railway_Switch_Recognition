#pragma once

#include "LIDARPT.h"
#include <pcl/common/common_headers.h>
#include "PointXYZDouble.h"

#include "etc/matrix.h"
#include <vector>
#include "LASFileReader.h"

#include "VTKWindowController.h"

#include "UTM.h"

using namespace System;

class DataConvertor
{
public:
	DataConvertor(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud);
	~DataConvertor();

public:
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;
	vector<PointXYZDouble> convertedPts;

	//convert ECEF to Geodetic (Geodetic to ECEF)
	double a, e2, a1, a2, a3, a4, a5, a6;

	void ConvertECEFToUTM(int zone);
	void ConvertUTMToECEF(int zone, bool southhemi);
	void SaveConvertedPoints(const char *filename);

	void ConvertECEFToUTM(int zone, UnmanagedProgressCallback callback);
	void ConvertUTMToECEF(int zone, bool southhemi, UnmanagedProgressCallback callback);
	void SaveConvertedPoints(const char *filename, UnmanagedProgressCallback callback);
	void SaveConvertedPoints(const char* filename, UnmanagedProgressCallback callback, const char* delim);
		
	void Ecef_to_geodetic(double x, double y, double z, double &lat, double &lon, double &altitude);
	void Geodetic_to_ecef(double lat, double lon, double alt, double &x, double &y, double &z);

};