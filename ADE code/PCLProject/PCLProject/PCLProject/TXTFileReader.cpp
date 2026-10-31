#include "TXTFileReader.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
//#include "lasreader.hpp"

#define RAIL_INTENSITY 100
#define RAIL_INTENSITY_LEFT 100
#define RAIL_INTENSITY_RIGHT 200


TXTFileReader::TXTFileReader()
{

}

TXTFileReader::~TXTFileReader()
{
}

pcl::PointCloud<PointXYZDouble>::Ptr TXTFileReader::ReadData(const char *fileName)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in);

	char Str[200];
	char Delim[] = " ";
	char *pTok = NULL;

	minPt.x = 1e100;
	minPt.y = 1e100;
	maxPt.x = -1e100;
	maxPt.y = -1e100;

	ifs.getline(Str, 200);
	int Num = atoi(Str);

//	while (!ifs.eof()) {
	for(int i=0;i<Num;i++){
		PointXYZDouble pt;

		ifs.getline(Str, 200);
		pTok = strtok(Str, Delim);
		pt.x = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.y = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.z = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.Intensity = atoi(pTok);

		minPt.x = MIN(minPt.x, pt.x);
		minPt.y = MIN(minPt.y, pt.y);
		maxPt.x = MAX(maxPt.x, pt.x);
		maxPt.y = MAX(maxPt.y, pt.y);

		point_cloud->points.push_back(pt);

	}
	
	ifs.close();

	loaded_point_cloud = point_cloud;

	return point_cloud;
}

pcl::PointCloud<PointXYZDouble>::Ptr TXTFileReader::ReadData(const char * fileName, UnmanagedProgressCallback callback)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in);

	char Str[200];
	char Delim[] = " ";
	char *pTok = NULL;

	minPt.x = 1e100;
	minPt.y = 1e100;
	maxPt.x = -1e100;
	maxPt.y = -1e100;

	ifs.getline(Str, 200);
	int Num = atoi(Str);
	size_t progress = 0;
	const int mod = 10;

	//	while (!ifs.eof()) {
	for (int i = 0; i<Num; i++) {
		PointXYZDouble pt;

		ifs.getline(Str, 200);
		pTok = strtok(Str, Delim);
		pt.x = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.y = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.z = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.Intensity = atoi(pTok);

		minPt.x = MIN(minPt.x, pt.x);
		minPt.y = MIN(minPt.y, pt.y);
		maxPt.x = MAX(maxPt.x, pt.x);
		maxPt.y = MAX(maxPt.y, pt.y);

		point_cloud->points.push_back(pt);

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, Num);
		}

	}

	ifs.close();

	loaded_point_cloud = point_cloud;
	callback(Num, Num);

	return point_cloud;
}

pcl::PointCloud<PointXYZDouble>::Ptr TXTFileReader::ReadDataCSV(const char* fileName, UnmanagedProgressCallback callback)
{
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud(new pcl::PointCloud<PointXYZDouble>);

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in);

	char Str[200];
	char Delim[] = ",";
	char* pTok = NULL;

	minPt.x = 1e100;
	minPt.y = 1e100;
	maxPt.x = -1e100;
	maxPt.y = -1e100;

	ifs.getline(Str, 200);
	int Num = atoi(Str);
	size_t progress = 0;
	const int mod = 10;

	//	while (!ifs.eof()) {
	for (int i = 0; i < Num; i++) {
		PointXYZDouble pt;

		ifs.getline(Str, 200);
		pTok = strtok(Str, Delim);
		pt.x = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.y = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.z = atof(pTok);
		pTok = strtok(NULL, Delim);
		pt.Intensity = atoi(pTok);

		minPt.x = MIN(minPt.x, pt.x);
		minPt.y = MIN(minPt.y, pt.y);
		maxPt.x = MAX(maxPt.x, pt.x);
		maxPt.y = MAX(maxPt.y, pt.y);

		point_cloud->points.push_back(pt);

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, Num);
		}

	}

	ifs.close();

	loaded_point_cloud = point_cloud;
	callback(Num, Num);

	return point_cloud;
}

pcl::PointCloud<PointXYZDouble>::Ptr TXTFileReader::LoadedData()
{
	return loaded_point_cloud;
}

void TXTFileReader::saveRailVector(const char *filename, vector<POINT3D> vtx)
{
	std::ofstream ofs(filename);
	ofs.setf(ios::fixed);
	ofs.setf(ios::showpoint);
	ofs.precision(3);

	ofs << vtx.size() << endl;
	for (int i = 0; i < vtx.size(); i++) {
		ofs << vtx[i].x << "," << vtx[i].y << "," << vtx[i].z << "," << vtx[i].PtType << endl;
	}

	ofs.close();

	printf("Save Done\n");
}

void TXTFileReader::saveRailVector(const char* filename, vector<POINT3D> vtx, vector<POINT3D> vtl, vector<POINT3D> vtr)
{
	std::ofstream ofs(filename);
	ofs.setf(ios::fixed);
	ofs.setf(ios::showpoint);
	ofs.precision(3);

	ofs << vtx.size() + vtl.size() + vtr.size() << endl;
	/*int tp = 0;
	for (int i = 0; i < vtl.size(); i++) {

		if (i + tp < vtx.size() && vtx[i + tp].PtType == 2) {
			ofs << vtx[i + tp].x << "," << vtx[i + tp].y << "," << vtx[i + tp].z << "," << vtx[i + tp].PtType << endl;
			tp++;
		}

		
		ofs << vtx[i + tp].x << "," << vtx[i + tp].y << "," << vtx[i + tp].z << "," << vtx[i + tp].PtType << endl;
		ofs << vtr[i].x << "," << vtr[i].y << "," << vtr[i].z << "," << 100 << endl;
	}*/

	for (int i = 0; i < vtx.size(); i++) {
		ofs << vtx[i].x << "," << vtx[i].y << "," << vtx[i].z << "," << vtx[i].PtType << endl;
	}

	for (int i = 0; i < vtl.size(); i++) {
		ofs << vtl[i].x << "," << vtl[i].y << "," << vtl[i].z << "," << RAIL_INTENSITY << endl;
	}

	for (int i = 0; i < vtr.size(); i++) {
		ofs << vtr[i].x << "," << vtr[i].y << "," << vtr[i].z << "," << RAIL_INTENSITY << endl;
	}


	ofs.close();

	printf("Save Done\n");
}

void TXTFileReader::saveRailVectorWithIndex(const char* filename, vector<POINT3D> vtx, vector<POINT3D> vtl, vector<POINT3D> vtr)
{
	std::ofstream ofs(filename);
	ofs.setf(ios::fixed);
	ofs.setf(ios::showpoint);
	ofs.precision(3);

	ofs << vtl.size() * 3 << endl;

	int tp = 0;
	for (int i = 0; i < vtl.size(); i++) {

		ofs << vtl[i].x << "," << vtl[i].y << "," << vtl[i].z << "," << RAIL_INTENSITY_LEFT << "," << i <<endl;

		if (vtx[i + tp].PtType == 2) {
			tp += 1;
		}

		ofs << vtx[i + tp].x << "," << vtx[i + tp].y << "," << vtx[i + tp].z << "," << vtx[i + tp].PtType << "," << i << endl;
		ofs << vtr[i].x << "," << vtr[i].y << "," << vtr[i].z << "," << RAIL_INTENSITY_RIGHT << "," << i << endl;
	}

	/*for (int i = 0; i < vtx.size(); i++) {
		ofs << vtx[i].x << "," << vtx[i].y << "," << vtx[i].z << "," << vtx[i].PtType << endl;
	}

	for (int i = 0; i < vtl.size(); i++) {
		ofs << vtl[i].x << "," << vtl[i].y << "," << vtl[i].z << "," << RAIL_INTENSITY_LEFT << endl;
	}

	for (int i = 0; i < vtr.size(); i++) {
		ofs << vtr[i].x << "," << vtr[i].y << "," << vtr[i].z << "," << RAIL_INTENSITY_RIGHT << endl;
	}*/


	ofs.close();

	printf("Save Done\n");
}
