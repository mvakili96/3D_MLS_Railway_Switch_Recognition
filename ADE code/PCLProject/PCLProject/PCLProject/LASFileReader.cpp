#include "LASFileReader.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
//#include "lasreader.hpp"


LASFileReader::LASFileReader()
{
	
}

LASFileReader::~LASFileReader()
{
	loaded_point_cloud.reset();
}

pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::ReadData(const char *fileName)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	liblas::Header header = reader.GetHeader();

	size_t numbers = 0;
	bool skip = true;

	while (reader.ReadNextPoint())
	{
		skip = !skip;

		liblas::Point const& p = reader.GetPoint();
	//	if (p.GetIntensity() > 100)
	//		continue;

		if (skip)
			continue;

		PointXYZDouble point(p.GetX(), p.GetY(), p.GetZ());
		point.Intensity = p.GetIntensity();
		point_cloud->points.push_back(point);
		numbers++;
	}

	loaded_point_cloud = point_cloud;

	ifs.close();

	size_t size_orig = header.GetPointRecordsCount(); 

	return point_cloud;
}

pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::ReadData(const char * fileName, UnmanagedProgressCallback callback)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	liblas::Header header = reader.GetHeader();

	size_t numbers = 0;
	size_t size_orig = header.GetPointRecordsCount();
	bool skip = true;

	const int mod = 10;

	while (reader.ReadNextPoint())
	{
		skip = !skip;

		liblas::Point const& p = reader.GetPoint();
		//	if (p.GetIntensity() > 100)
		//		continue;

		numbers++;

		if (numbers % mod == 0)
		{
			callback(numbers, size_orig);
		}

		if (skip)
			continue;

		PointXYZDouble point(p.GetX(), p.GetY(), p.GetZ());
		point.Intensity = p.GetIntensity();
		point_cloud->points.push_back(point);
	}

	loaded_point_cloud = point_cloud;

	ifs.close();
	callback(size_orig, size_orig);

	return point_cloud;
}

pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::LoadedData()
{
	return loaded_point_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LASFileReader::DecreaseDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	size_t old_points = cloud->points.size();

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*filtered_cloud);

	size_t new_points = filtered_cloud->points.size();

	return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LASFileReader::Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	float theta = M_PI / 4;
	Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
	
	Eigen::Vector4d centroid = Eigen::Vector4d::Identity();
	pcl::compute3DCentroid(*cloud, centroid);

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << -1 * centroid[0], -1 * centroid[1], -1 * centroid[2];

	// The same rotation matrix as before; theta radians arround Z axis
	//transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

	return transformed_cloud;
}

PointXYZDouble LASFileReader::maxPoint(const char *fileName)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	liblas::Header header = reader.GetHeader();

	PointXYZDouble point(header.GetExtent().maxx(), header.GetExtent().maxy(), header.GetExtent().maxz());

	ifs.close();

	return point;
}

PointXYZDouble LASFileReader::minPoint(const char *fileName)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	liblas::Header header = reader.GetHeader();

	PointXYZDouble point(header.GetExtent().minx(), header.GetExtent().miny(), header.GetExtent().minz());

	ifs.close();

	return point;
}