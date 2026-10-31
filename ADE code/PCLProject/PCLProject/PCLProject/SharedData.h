#pragma once
#include <iostream>
#include <pcl/common/common_headers.h>
#include "PointXYZDouble.h"

#include "etc/matrix.h"
#include <vector>
#include "LASFileReader.h"

#include "VTKWindowController.h"
#include "HashMap.h"

class SharedData
{
private:
	static bool instanceFlag;
	static SharedData *single;

	SharedData()
	{
		//private constructor
	}

public:
	static SharedData* getInstance();
	void method();

	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;
	double centerOfMass[3];

	HashMap filesMap;

	~SharedData()
	{
		instanceFlag = false;
	}
};

