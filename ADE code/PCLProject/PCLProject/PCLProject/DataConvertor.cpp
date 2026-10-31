#include "DataConvertor.h"


#include <vtkCylinderSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkSimplePointsWriter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCenterOfMass.h>
#include <vtkLine.h>

#include <iostream>
#include <fstream>


DataConvertor::DataConvertor(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud)
{
	this->point_cloud = point_cloud;
	a = 6378137.0;
	e2 = 6.6943799901377997e-3;
	a1 = 4.2697672707157535e+4;
	a2 = 1.8230912546075455e+9;
	a3 = 1.4291722289812413e+2;
	a4 = 4.5577281365188637e+9;
	a5 = 4.2840589930055659e+4;
	a6 = 9.9330562000986220e-1;

}
DataConvertor::~DataConvertor()
{

}

void DataConvertor::ConvertECEFToUTM(int zone) 
{
	for(int i = 0; i < point_cloud->points.size(); i++) 
		convertedPts.push_back(point_cloud->points[i]);

	//convert from ecef to geodetic 
	for(int i = 0; i < convertedPts.size(); i++) {
		double x = convertedPts[i].x;
		double y = convertedPts[i].y;
		double z = convertedPts[i].z;

		double lat;
		double lon;
		double alt;
		Ecef_to_geodetic(x, y, z, lat, lon, alt);
		convertedPts[i].x = lat * 180 / PI;
		convertedPts[i].y = lon * 180 / PI;
		convertedPts[i].z = alt;

	//	if (i == 0) {
	//		printf("%0.6f,%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", x, y, z, lat * 180 / PI, lon * 180 / PI, alt);
	//	}
	}

	//convert from geodetic to UTM
	for (int i = 0; i < convertedPts.size(); i++) {
		FLOAT lat = convertedPts[i].x;
		FLOAT lon = convertedPts[i].y;

		FLOAT x,y;
		LatLonToUTMXY(lat, lon, zone, x, y);

		convertedPts[i].x = x;
		convertedPts[i].y = y;
	//	if (i == 0) {
	//		printf("%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", lat, lon, convertedPts[i].x, convertedPts[i].y, convertedPts[i].z);
	//	}
		
	}

	printf("Conversion Done\n");
}

void DataConvertor::ConvertECEFToUTM(int zone, UnmanagedProgressCallback callback)
{
	size_t totalPoints = point_cloud->points.size() * 3;
	size_t progress = 0;
	const int mod = 10;

	for (int i = 0; i < point_cloud->points.size(); i++)
	{
		convertedPts.push_back(point_cloud->points[i]);

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	//convert from ecef to geodetic 
	for (int i = 0; i < convertedPts.size(); i++) {
		double x = convertedPts[i].x;
		double y = convertedPts[i].y;
		double z = convertedPts[i].z;

		double lat;
		double lon;
		double alt;
		Ecef_to_geodetic(x, y, z, lat, lon, alt);
		convertedPts[i].x = lat * 180 / PI;
		convertedPts[i].y = lon * 180 / PI;
		convertedPts[i].z = alt;

		//	if (i == 0) {
		//		printf("%0.6f,%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", x, y, z, lat * 180 / PI, lon * 180 / PI, alt);
		//	}

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	//convert from geodetic to UTM
	for (int i = 0; i < convertedPts.size(); i++) {
		FLOAT lat = convertedPts[i].x;
		FLOAT lon = convertedPts[i].y;

		FLOAT x, y;
		LatLonToUTMXY(lat, lon, zone, x, y);

		convertedPts[i].x = x;
		convertedPts[i].y = y;
		//	if (i == 0) {
		//		printf("%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", lat, lon, convertedPts[i].x, convertedPts[i].y, convertedPts[i].z);
		//	}

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}

	}

	printf("Conversion Done\n");
}

void DataConvertor::ConvertUTMToECEF(int zone, bool southhemi)
{
	for (int i = 0; i < point_cloud->points.size(); i++)
		convertedPts.push_back(point_cloud->points[i]);

	//convert from UTM to geodetic
	for (int i = 0; i < convertedPts.size(); i++) {
		FLOAT x = convertedPts[i].x;
		FLOAT y = convertedPts[i].y;
		FLOAT lat, lon;
		UTMXYToLatLon(x, y, zone, southhemi, lat, lon);
			
		convertedPts[i].x = lat;
		convertedPts[i].y = lon;

	//	if (i == 0) {
	//		printf("%0.6f,%0.6f->%0.6f,%0.6f\n", x, y, lat * 180 / PI, lon * 180 / PI);
	//	}
	}

	//convert from geodetic to ECEF
	for (int i = 0; i < convertedPts.size(); i++) {
		double lat = convertedPts[i].x;
		double lon = convertedPts[i].y;
		double alt = convertedPts[i].z;
		double x, y, z;
		Geodetic_to_ecef(lat, lon, alt, x, y, z);

		convertedPts[i].x = x;
		convertedPts[i].y = y;
		convertedPts[i].z = z;

	//	if (i == 0) {
	//		printf("%0.6f,%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", lat * 180 / PI, lon * 180 / PI, alt, x, y, z);
	//	}
	}
	printf("Conversion Done\n");
}

void DataConvertor::ConvertUTMToECEF(int zone, bool southhemi, UnmanagedProgressCallback callback)
{


	size_t totalPoints = point_cloud->points.size() * 3;
	size_t progress = 0;
	const int mod = 10;

	for (int i = 0; i < point_cloud->points.size(); i++) 
	{
		convertedPts.push_back(point_cloud->points[i]);

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	//convert from UTM to geodetic
	for (int i = 0; i < convertedPts.size(); i++) {
		FLOAT x = convertedPts[i].x;
		FLOAT y = convertedPts[i].y;
		FLOAT lat, lon;
		UTMXYToLatLon(x, y, zone, southhemi, lat, lon);

		convertedPts[i].x = lat;
		convertedPts[i].y = lon;

		//	if (i == 0) {
		//		printf("%0.6f,%0.6f->%0.6f,%0.6f\n", x, y, lat * 180 / PI, lon * 180 / PI);
		//	}

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	//convert from geodetic to ECEF
	for (int i = 0; i < convertedPts.size(); i++) {
		double lat = convertedPts[i].x;
		double lon = convertedPts[i].y;
		double alt = convertedPts[i].z;
		double x, y, z;
		Geodetic_to_ecef(lat, lon, alt, x, y, z);

		convertedPts[i].x = x;
		convertedPts[i].y = y;
		convertedPts[i].z = z;

		//	if (i == 0) {
		//		printf("%0.6f,%0.6f,%0.6f->%0.6f,%0.6f,%0.6f\n", lat * 180 / PI, lon * 180 / PI, alt, x, y, z);
		//	}

		progress++;
		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	printf("Conversion Done\n");
}

void DataConvertor::Ecef_to_geodetic(double x, double y, double z, double &lat, double &lon, double &altitude)
{
	double zp = fabs(z);
	double w2 = x*x + y*y;
	double w = sqrt(w2);
	double r2 = w2 + z*z;
	double r = sqrt(r2);
	lon = atan2(y, x);       //Lon (final)
	double s2 = z*z / r2;
	double c2 = w2 / r2;
	double u = a2 / r;
	double v = a3 - a4 / r;
	double s, ss, c;
	if (c2 > 0.3) {
		s = (zp / r)*(1.0 + c2*(a1 + u + s2*v) / r);
		lat = asin(s);      //Lat
		ss = s*s;
		c = sqrt(1.0 - ss);
	}
	else {
		c = (w / r)*(1.0 - s2*(a5 - u - c2*v) / r);
		lat = acos(c);      //Lat
		ss = 1.0 - c*c;
		s = sqrt(ss);
	}
	double g = 1.0 - e2*ss;
	double rg = a / sqrt(g);
	double rf = a6*rg;
	u = w - rg*c;
	v = zp - rf*s;
	double f = c*u + s*v;
	double m = c*v - s*u;
	double p = m / (rf / g + f);
	lat = lat + p;      //Lat
	altitude = f + m*p / 2.0;     //Altitude
	if (z < 0.0) {
		lat *= -1.0;     //Lat
	}
	
}

void DataConvertor::Geodetic_to_ecef(double lat, double lon, double alt, double &x, double &y, double &z)
{
	double n = a / sqrt(1 - e2*sin(lat)*sin(lat));
	x = (n + alt)*cos(lat)*cos(lon);    //ECEF x
	y = (n + alt)*cos(lat)*sin(lon);    //ECEF y
	z = (n*(1 - e2) + alt)*sin(lat);    //ECEF z
}

void DataConvertor::SaveConvertedPoints(const char *filename)
{
	std::ofstream ofs(filename);
	ofs.setf(ios::fixed);
	ofs.setf(ios::showpoint);
	ofs.precision(3);

	ofs << convertedPts.size() << endl;
	for (int i = 0; i < convertedPts.size(); i++) {
		ofs << convertedPts[i].x<<" "<<convertedPts[i].y<<" "<< convertedPts[i].z <<" "<<convertedPts[i].Intensity<< endl;
	}
	
	ofs.close();

	printf("Save Done\n");
	
}

void DataConvertor::SaveConvertedPoints(const char* filename, UnmanagedProgressCallback callback)
{
	SaveConvertedPoints(filename, callback, " ");
}

void DataConvertor::SaveConvertedPoints(const char* filename, UnmanagedProgressCallback callback, const char* delim)
{
	std::ofstream ofs(filename);
	ofs.setf(ios::fixed);
	ofs.setf(ios::showpoint);
	ofs.precision(3);

	ofs << convertedPts.size() << endl;

	size_t totalPoints = convertedPts.size();
	size_t progress = 0;
	const int mod = 10;

	for (int i = 0; i < convertedPts.size(); i++) 
	{
		ofs << convertedPts[i].x << delim << convertedPts[i].y << delim << convertedPts[i].z << delim << convertedPts[i].Intensity << endl;
		progress++;

		if (progress % mod == 0)
		{
			callback(progress, totalPoints);
		}
	}

	ofs.close();

	printf("Save Done\n");

}