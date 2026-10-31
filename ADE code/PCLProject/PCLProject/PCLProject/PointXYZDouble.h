#pragma once
#define PCL_NO_PRECOMPILE
#define generic __identifier(generic)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#undef generic

struct PointXYZDouble
{
	union EIGEN_ALIGN16 {
		double data[4];
		struct {
			double x;
			double y;
			double z;
		};
	};

	int ClassCode;
	int	SegmentID;
	int Intensity;

	inline PointXYZDouble()
	{
		x = y = z = 0.0f;
		data[3] = 1.0f;
		ClassCode = 0;
		SegmentID = -1;
		Intensity = 0;
	}

	inline PointXYZDouble(double _x, double _y, double _z)
	{
		x = _x; y = _y; z = _z;
		data[3] = 1.0f;
		ClassCode = 0;
		SegmentID = -1;
		Intensity = 0;
	}

	inline PointXYZDouble(const PointXYZDouble *p)
	{
		x = p->x; y = p->y; z = p->z;
		data[3] = 1.0f;
		ClassCode = p->ClassCode;
		SegmentID = p->SegmentID;
		Intensity = p->Intensity;
	}

	bool operator == (PointXYZDouble &rhs) 
	{
		double fPrecision = 1e-5;
		if (fabs(x - rhs.x)<fPrecision && fabs(y - rhs.y)<fPrecision && fabs(z - rhs.z)<fPrecision)
			return true;
		else
			return false;
		//return x == rhs.x && y == rhs.y && z == rhs.z;
	}


	bool IsEqual(PointXYZDouble rhs)
	{
		double fPrecision = 1e-5;
		if (fabs(x - rhs.x)<fPrecision && fabs(y - rhs.y)<fPrecision && fabs(z - rhs.z)<fPrecision)
			return true;
		else
			return false;
	}
	
	bool isNull()
	{
		return x == 0 && y == 0 && z == 0;
	}

} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZDouble,           // here we assume a XYZ + "test" (as fields)
(double, x, x)
(double, y, y)
(double, z, z)
(int, ClassCode, ClassCode)
(int, SegmentID, SegmentID)
(int, Intensity, Intensity))