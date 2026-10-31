 #include "RailwayExtraction.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>

#include "SharedData.h"

using namespace std;

RailwayExtraction::RailwayExtraction(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud)
{
	m_BoundMax.x = -1e100;
	m_BoundMax.y = -1e100;
	m_BoundMax.z = -1e100;
	m_BoundMin.x = 1e100;
	m_BoundMin.y = 1e100;
	m_BoundMin.z = 1e100;

	m_skip_rate = 1;

	m_pGrid = NULL;
	// m_ClassView = 0;
	m_GridSize = 1;
	m_gWidth = 0.42;	//0.42
	m_gLength = 1;	//1
	m_NumGrid = 0;
	m_ThRansac = 0.06;
	m_bInit = true;
	m_distFromSp = 0;
	m_TrkGuage = 1.5;
	m_ThMinTrkPts = 5;
	m_ThMinPts = 20;
	m_wMDL = 0.4;
	m_ThDistGrouping = 0.15;
	m_ThMinIntensityForTrack = 0;
	m_ThMaxIntensityForTrack = 1500;


	this->point_cloud = point_cloud;


	//Template
	DefineTemplates();
}

void RailwayExtraction::updateExtractionParameters(ExtractionParameters parameters)
{
	//m_GridSize = 1;
	m_gWidth = parameters.trackWindowWidth;
	m_gLength = parameters.trackWindowLength;
	//m_NumGrid = 0;
	m_ThRansac = parameters.maxDistanceForRANSAC;
	//m_bInit = true;
	//m_distFromSp = 0;
	m_TrkGuage = parameters.trackGuage;
	m_ThMinTrkPts = parameters.minTrackPoints;
	m_ThMinPts = parameters.minPoints;
	m_wMDL = parameters.weightParameterForMDL;
	m_ThDistGrouping = parameters.distanceGrouping;
	m_ThMinIntensityForTrack = parameters.minIntensityRangeOfTrack;
	m_ThMaxIntensityForTrack = parameters.maxIntensityRangeOfTrack;
}

ExtractionParameters RailwayExtraction::getExtractionParameters()
{
	ExtractionParameters parameters;

	//m_GridSize = 1;
	parameters.trackWindowWidth = m_gWidth;
	parameters.trackWindowLength = m_gLength;
	//m_NumGrid = 0;
	parameters.maxDistanceForRANSAC = m_ThRansac;
	//m_bInit = true;
	//m_distFromSp = 0;
	parameters.trackGuage = m_TrkGuage;
	parameters.minTrackPoints = m_ThMinTrkPts;
	parameters.minPoints = m_ThMinPts;
	parameters.weightParameterForMDL = m_wMDL;
	parameters.distanceGrouping = m_ThDistGrouping;
	parameters.minIntensityRangeOfTrack = m_ThMinIntensityForTrack;
	parameters.maxIntensityRangeOfTrack = m_ThMaxIntensityForTrack;

	return parameters;
}


RailwayExtraction::~RailwayExtraction()
{
}


void RailwayExtraction::setInitialLocation()
{
}

void RailwayExtraction::updateMaxMin(PointXYZDouble maxPoint, PointXYZDouble minPoint)
{
	m_BoundMax.x = maxPoint.x;
	m_BoundMax.y = maxPoint.y;
	m_BoundMax.z = maxPoint.z;

	m_BoundMin.x = minPoint.x;
	m_BoundMin.y = minPoint.y;
	m_BoundMin.z = minPoint.z;
}

void RailwayExtraction::updateMaxMin(double *maxPoint, double *minPoint)
{
	m_BoundMax.x = maxPoint[0];
	m_BoundMax.y = maxPoint[1];
	m_BoundMax.z = maxPoint[2];

	m_BoundMin.x = minPoint[0];
	m_BoundMin.y = minPoint[1];
	m_BoundMin.z = minPoint[2];
}

void RailwayExtraction::updateMaxMin()
{

}

void RailwayExtraction::updateMaxMinFromReader(LASFileReader *reader, const char *filename)
{
	readMaxMinFromReader(reader, filename);
}

void RailwayExtraction::readMaxMinFromReader(LASFileReader *reader, const char *filename)
{
	PointXYZDouble maxPoint = reader->maxPoint(filename);
	PointXYZDouble minPoint = reader->minPoint(filename);

	updateMaxMin(maxPoint, minPoint);
}

void RailwayExtraction::readMaxMinFromTXTReader(TXTFileReader *txtReader)
{
	PointXYZDouble maxPoint = txtReader->maxPt;
	PointXYZDouble minPoint = txtReader->minPt;

	updateMaxMin(maxPoint, minPoint);
}

/**
 * @fn	void RailwayExtraction::generateGrid()
 *
 * @brief	Generates grids
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 */

void RailwayExtraction::generateGrid()
{
	//updateMaxMin();

	double _gSize = m_GridSize;
	m_NumX = max((double)1, ceil((m_BoundMax.x - m_BoundMin.x) / (double)_gSize));
	m_NumY = max((double)1, ceil((m_BoundMax.y - m_BoundMin.y) / (double)_gSize));

	m_NumGrid = m_NumX*m_NumY;
	m_pGrid = new Grid[m_NumGrid];

	size_t size = point_cloud->points.size();
	for (size_t i = 0; i<point_cloud->points.size(); i++) {
		int x, y, idx;
		x = min(m_NumX - 1, floor(point_cloud->points[i].x - m_BoundMin.x) / (double)_gSize);
		y = min(m_NumY - 1, floor(point_cloud->points[i].y - m_BoundMin.y) / (double)_gSize);
		idx = y*m_NumX + x;

		m_pGrid[idx].MemPnts.push_back(&(point_cloud->points[i]));
		m_pGrid[idx].m_nX = x;
		m_pGrid[idx].m_nY = y;

		if (i % 100 == 0) {
			progressCallback(i, size);
		}
	}

	for (size_t i = 0; i<m_NumY; i++) {
		for (size_t j = 0; j<m_NumX; j++) {
			int idx = i*m_NumX + j;
			m_pGrid[idx].minX = m_BoundMin.x + j*_gSize;
			m_pGrid[idx].maxX = m_BoundMin.x + (j + 1)*_gSize;
			m_pGrid[idx].minY = m_BoundMin.y + i*_gSize;
			m_pGrid[idx].maxY = m_BoundMin.y + (i + 1)*_gSize;
		}

		progressCallback(i+1, m_NumY);
	}

	printf("Grid Generation Done!!\n");
	printf("Grid Num: %lf\n", m_NumGrid);
	progressCallback(0, 1);
}

/**
 * @fn	void RailwayExtraction::OnRailextractionSetinit()
 *
 * @brief	Initial parameter setting
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 */

void RailwayExtraction::OnRailextractionSetinit()
{
	// TODO: Add your command handler code here

	POINT3D pt;
//	int ang = -1.5;
//	int mode = 4;
//	pt.x = 513195.62;
//	pt.y = 5458237.77;
//	pt.z = -9.2559631349317831e+61;

	int ang = 280;
	int mode = 4;
//	pt.x = -727.81;
//	pt.y = 1950;
//	pt.z = -9.2559631349317831e+61;

	Setinitiallocation(mode, pt, ang);

	printf("Initialization Done!!\n");
	//visualization
	//m_ClassView = 2;
	//UpdateAllViews(NULL);

	updateView(1);
}

void RailwayExtraction::OnRailextractionSetinit(int mode, POINT3D pt, float angle)
{
//	m_updateBnd.clear();
	m_TrackSegList.clear();
	m_trackLayers.clear();

	currentLayer = new TrackLayer();
	Setinitiallocation(mode, pt, angle);

	printf("Initialization Done!!\n");
	//visualization
	//m_ClassView = 2;
	//UpdateAllViews(NULL);
	m_trackLayers.push_back(currentLayer);
	updateView(1);
}

void RailwayExtraction::OnRailextractionTrackingRailway()
{
	// TODO: Add your command handler code here
	printf("Railway Tracking Start!!\n");
	bool mainTrkflg;
	mainTrkflg = true;
	int TrkDirection = 1;
	
	Trackingrailway(TrkDirection, mainTrkflg);	//0:right direction, 1: left direction

	////////Test add point
	/*POINT3D testPt;
	testPt.x = 513203.98;
	testPt.y = 5458240.78;
	testPt.z = 6.562;
	if (m_trackLayers.size() > 0) {
		AddPointTofootOfPependicularOnRailVtx(m_trackLayers[0]->m_RailVecCenter, testPt, 1);
	}*/

	//////// until here

	vector<ToePt> toes = m_ToePts;
	//additional track
	printf("Toe points start\n");
	for (int i = 0; i<toes.size(); i++){
		toes[i].used = false;
		printf("%0.3f %0.3f\n", toes[i].CenterPt.x, toes[i].CenterPt.y);
	}
	printf("Toe points End\n");
		

	
	mainTrkflg = false;
	for (int i = 0; i<toes.size(); i++) {
		if (toes[i].used == true)
			continue;
		//	if(toes[i].TrkType != 1 && toes[i].TrkType != 2 && toes[i].TrkType != 5 && toes[i].TrkType != 6)
		//		continue;
		
		currentLayer = new TrackLayer();
		m_trackLayers.push_back(currentLayer);

		if (toes[i].TrkType == 2 || toes[i].TrkType == 3 || toes[i].TrkType == 6 || toes[i].TrkType == 7
			|| toes[i].TrkType == 8 || toes[i].TrkType == 9 || toes[i].TrkType == 10 || toes[i].TrkType == 11
			|| toes[i].TrkType == 14 || toes[i].TrkType == 15 || toes[i].TrkType == 16
			|| toes[i].TrkType == 17 || toes[i].TrkType == 18 || toes[i].TrkType == 19)
			continue;

		printf("%d %0.3f\n", toes[i].TrkMode, toes[i].TrkAng);
		printf("%0.3f %0.3f %0.3f\n", toes[i].RightPt.x, toes[i].RightPt.y, toes[i].RightPt.z);

		POINT3D pt;
		pt.SetPoint3D(toes[i].RightPt.x, toes[i].RightPt.y, toes[i].RightPt.y);
		double trkang = toes[i].TrkAng;
		int trkmode = toes[i].TrkMode;

		if (Setinitiallocation(trkmode, pt, trkang) == true) {
			printf("Initialization Done!!\n");
			if (TrkDirection == 0)
				Trackingrailway(1, false);
			else
				Trackingrailway(0, false);
		}
		else
			printf("Initialization fail!!\n");

		toes[i].used = true;
	}

	for (int n = 0; n < m_trackLayers.size(); n++)
	{
		TrackLayer *layer = m_trackLayers[n];
		for (int i = 0; i < layer->m_ToePts.size(); i++) {
			POINT3D pt;
			pt = layer->m_ToePts[i].CenterPt;
			AddPointTofootOfPependicularOnRailVtx(m_trackLayers[n]->m_RailVecCenter, pt, 2);
		}
	}
	


	printf("Railway Tracking Done!!\n");
	updateView(1);
}

/**
 * @fn	bool RailwayExtraction::Setinitiallocation(int inMode, POINT3D inPt, double inAng)
 *
 * @brief	Seting initial locations of trackwindows
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	inMode	The in mode.
 * @param	inPt  	The in point.
 * @param	inAng 	The in angle.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::Setinitiallocation(int inMode, POINT3D inPt, double inAng)
{
	double _gSize = m_GridSize;
	double _LnDist = m_TrkGuage;
	m_bInit = true;
	m_distFromSp = 0;

	TrackSegment initSeg;
	int mode = inMode;	//quadrant 4

	initSeg.m_Track1.CenterPt = inPt;
	initSeg.m_Track1.ori = inAng;
	initSeg.SetTrkGauge(m_TrkGuage);

	//Set pair track segment window
	double ang = (initSeg.m_Track1.ori) * PI / 180;
	if (mode == 1)
		initSeg.m_Track2.CenterPt.SetPoint3D(initSeg.m_Track1.CenterPt.x - initSeg.m_D * fabs(sin(ang)), initSeg.m_Track1.CenterPt.y + initSeg.m_D * fabs(cos(ang)), 0);
	else if (mode == 2)
		initSeg.m_Track2.CenterPt.SetPoint3D(initSeg.m_Track1.CenterPt.x - initSeg.m_D * fabs(sin(ang)), initSeg.m_Track1.CenterPt.y - initSeg.m_D * fabs(cos(ang)), 0);
	else if (mode == 3)
		initSeg.m_Track2.CenterPt.SetPoint3D(initSeg.m_Track1.CenterPt.x + initSeg.m_D * fabs(sin(ang)), initSeg.m_Track1.CenterPt.y - initSeg.m_D * fabs(cos(ang)), 0);
	else
		initSeg.m_Track2.CenterPt.SetPoint3D(initSeg.m_Track1.CenterPt.x + initSeg.m_D * fabs(sin(ang)), initSeg.m_Track1.CenterPt.y + initSeg.m_D * fabs(cos(ang)), 0);

	initSeg.m_Track2.ori = initSeg.m_Track1.ori;
	initSeg.SwapWnd(mode);
	initSeg.m_Track1.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
	initSeg.m_Track2.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
	initSeg.GetCenterOfTwoTracks();

	//Assign member points
	vector<int> neighborIdx = GetNeighborIdx(initSeg.m_Center.x, initSeg.m_Center.y, m_NumX, m_NumY, _gSize, 5);
	vector<PointXYZDouble*> rgnPts;
	for (int i = 0; i<neighborIdx.size(); i++) {
		if (neighborIdx[i]<0 || neighborIdx[i] >= m_NumGrid)
			continue;
		for (int j = 0; j<m_pGrid[neighborIdx[i]].MemPnts.size(); j++) {
			rgnPts.push_back(m_pGrid[neighborIdx[i]].MemPnts[j]);
		}
	}
	initSeg.m_Track1.SetTrackWindowMemPnts(rgnPts);
	initSeg.m_Track2.SetTrackWindowMemPnts(rgnPts);

	//Compute GMM parameter
	vector<PointXYZDouble*> tmpPts1 = initSeg.m_Track1.MemPnts;
	vector<PointXYZDouble*> tmpPts2 = initSeg.m_Track2.MemPnts;
	vector<GMMPara> paras1, paras2;
	ComputeGMMfeature(tmpPts1, paras1, 3);
	ComputeGMMfeature(tmpPts2, paras2, 3);

	//Clustering by GMM parameter
	vector<vector<PointXYZDouble*>> RailPts1, RailPts2;
	ClusteringByGMM(tmpPts1, paras1, RailPts1);
	ClusteringByGMM(tmpPts2, paras2, RailPts2);

	vector<PointXYZDouble*> HeadPts1 = RailPts1[RailPts1.size() - 1];
	vector<PointXYZDouble*> HeadPts2 = RailPts2[RailPts2.size() - 1];

	//stop condition (if track points are not detected)
	if (HeadPts1.size() == 0 || HeadPts2.size() == 0) {
		printf("No head points !!\n");
		return false;
	}
	//compute height of initial track points
	sort(HeadPts1.begin(), HeadPts1.end(), HigherZValue);
	sort(HeadPts2.begin(), HeadPts2.end(), HigherZValue);

	int HgtIdx1 = (int)(HeadPts1.size()*0.15);
	int HgtIdx2 = (int)(HeadPts2.size()*0.15);

	m_InitTrkHgt1 = HeadPts1[HgtIdx1]->z;
	m_InitTrkHgt2 = HeadPts2[HgtIdx2]->z;

	//visualization
	//	for(int i=0;i<HeadPts1.size();i++)
	//		HeadPts1[i]->ClassCode = 5;
	//	for(int i=0;i<HeadPts2.size();i++)
	//		HeadPts2[i]->ClassCode = 5;

	//track vector extraction
	Line3D tmpLn1, tmpLn2, tmpLn3;
	LineExtractionByRansac(HeadPts1, 0.04, tmpLn1);
	LineExtractionByRansac(HeadPts2, 0.04, tmpLn2);
	tmpLn1.SetEndingByMode(mode);
	tmpLn2.SetEndingByMode(mode);

	//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",
	//		tmpLn1.BeginPt.x,tmpLn1.BeginPt.y,tmpLn1.BeginPt.z,tmpLn1.EndPt.x,tmpLn1.EndPt.y,tmpLn1.EndPt.z);
	//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",
	//		tmpLn2.BeginPt.x,tmpLn2.BeginPt.y,tmpLn2.BeginPt.z,tmpLn2.EndPt.x,tmpLn2.EndPt.y,tmpLn2.EndPt.z);

	int mode1 = GetDirectionMode(tmpLn1.EndPt.x, tmpLn1.EndPt.y, tmpLn1.BeginPt.x, tmpLn1.BeginPt.y);
	int mode2 = GetDirectionMode(tmpLn2.EndPt.x, tmpLn2.EndPt.y, tmpLn2.BeginPt.x, tmpLn2.BeginPt.y);
	AdjustTwoLines(tmpLn1, tmpLn2, mode1, mode2, mode, _LnDist);

	//center track vector
	//	tmpLn3.BeginPt = (tmpLn1.BeginPt + tmpLn2.BeginPt) / 2;
	//	tmpLn3.EndPt = (tmpLn1.EndPt + tmpLn2.EndPt) / 2;
	//	tmpLn3.GetLineParameter();

	//	double fdist = DistanceBetweenPoints2D(tmpLn3.BeginPt, tmpLn3.EndPt);
	//	m_distFromSp = m_distFromSp + fdist;

	currentLayer->m_RailVec1.push_back(tmpLn1.CenterPt);
	currentLayer->m_RailVec2.push_back(tmpLn2.CenterPt);
	currentLayer->m_RailVecCenter.push_back((tmpLn1.CenterPt + tmpLn2.CenterPt) / 2);


	//update inital model vector
	initSeg.m_Track1.ori = tmpLn1.Ang; // USE THIS ANGLE
	initSeg.m_Track1.CenterPt = tmpLn1.CenterPt; //USE THIS X, Y
	initSeg.m_Track1.MemLn = tmpLn1;
	mode = GetDirectionMode(tmpLn1.EndPt.x, tmpLn1.EndPt.y, tmpLn1.BeginPt.x, tmpLn1.BeginPt.y);
	initSeg.m_Track1.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
	initSeg.m_Track2.ori = tmpLn2.Ang;
	initSeg.m_Track2.CenterPt = tmpLn2.CenterPt;
	initSeg.m_Track2.MemLn = tmpLn2;
	mode = GetDirectionMode(tmpLn2.EndPt.x, tmpLn2.EndPt.y, tmpLn2.BeginPt.x, tmpLn2.BeginPt.y); // USE THIS MODE
	initSeg.m_Track2.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
	initSeg.GetCenterOfTwoTracks();
	initSeg.GetDistanceBetweenTwoCenter();
	initSeg.distFromSp = m_distFromSp;

	//set Z value of bounding box
	for (int i = 0; i<initSeg.m_Track1.vtx.size(); i++)
		initSeg.m_Track1.vtx[i].z = initSeg.m_Track1.MemLn.BeginPt.z;
	for (int i = 0; i<initSeg.m_Track2.vtx.size(); i++)
		initSeg.m_Track2.vtx[i].z = initSeg.m_Track2.MemLn.BeginPt.z;

	//Visualization (updated BNDs)
	vector<Line3D> tmpUpdateLns;
	GetLnsFromVtx(initSeg.m_Track1.vtx, tmpUpdateLns);
	for (int i = 0; i<tmpUpdateLns.size(); i++)
		currentLayer->m_updateBnd.push_back(tmpUpdateLns[i]);
	GetLnsFromVtx(initSeg.m_Track2.vtx, tmpUpdateLns);
	for (int i = 0; i<tmpUpdateLns.size(); i++)
		currentLayer->m_updateBnd.push_back(tmpUpdateLns[i]);

	//Save track segment
	//m_TrackSegList.push_back(initSeg);
	currentLayer->m_TrackSegList.push_back(initSeg);

	initialLocation.x = initSeg.m_Track1.CenterPt.x; 
	initialLocation.y = initSeg.m_Track1.CenterPt.y;
	initialLocation.z = initSeg.m_Track1.CenterPt.z;

	//POINT3D centerPt = (tmpLn1.CenterPt + tmpLn2.CenterPt) / 2

	initialLocation.second_x = initSeg.m_Track2.CenterPt.x;
	initialLocation.second_y = initSeg.m_Track2.CenterPt.y;
	initialLocation.second_z = initSeg.m_Track2.CenterPt.z;

	initialLocation.angle = tmpLn1.Ang;
	initialLocation.mode = mode;

	////////////
	//Loop region
	//	double aveHgtOfGround = (aveHgtOfGround1+aveHgtOfGround2)/2;
	//	double aveHgtOfTrack = (aveHgtOfTrack1+aveHgtOfTrack2)/2;
	//	Line3D refLn = tmpLn1;
	//	DetectLoopCross(initSeg,rgnPts,aveHgtOfGround,aveHgtOfTrack,refLn,m_LoopCross);
	//////////////////

	return true;
}

/**
 * @fn	void RailwayExtraction::Trackingrailway(int TrackDirection, bool mainTrkflg)
 *
 * @brief	Rail tracking
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	TrackDirection	The track direction.
 * @param	mainTrkflg	  	True for main track.
 */

void RailwayExtraction::Trackingrailway(int TrackDirection, bool mainTrkflg)
{
	if (currentLayer->m_TrackSegList.size() == 0)
		return;

	TrackSegment curSeg = currentLayer->m_TrackSegList[currentLayer->m_TrackSegList.size() - 1];
	if (mainTrkflg == false) {
		m_distFromSp = 0;
		m_LoopCross.clear();
	}

	double _gSize = m_GridSize;
	int iter = 0;
	int mode;

	//Kalman filter variables for line 
	int nX = 3;
	Matrix<double> curPt1(nX, 1, 0), curPt2(nX, 1, 0);
	Matrix<double> curP1(nX, nX, 0), curP2(nX, nX, 0);
	Matrix<double> predPt1(nX, 1, 0), predPt2(nX, 1, 0);
	Matrix<double> predP1(nX, nX, 0), predP2(nX, nX, 0);
	Matrix<double> obsPt1(nX, 1, 0), obsPt2(nX, 1, 0);
	Matrix<double> hatPt1(nX, 1, 0), hatPt2(nX, 1, 0);
	Matrix<double> hatP1(nX, nX, 0), hatP2(nX, nX, 0);

	bool flg = true;
	vector<PointXYZDouble*> rgnPts;
	double curTrkHgt1, curTrkHgt2, nextTrkHgt1, nextTrkHgt2;
	TrackStack trkStk;		//for turnout region recogition

	while (flg == true) {
		//	for(int ii=0;ii<1;ii++){
		if (m_bInit == true) {
			POINT3D nPt = curSeg.m_Track1.MemLn.EndPt;
			POINT3D Pt = curSeg.m_Track1.MemLn.BeginPt;
			mode = GetDirectionMode(nPt.x, nPt.y, Pt.x, Pt.y);
			for (int i = 0; i < nX; i++) {
				curP1(i, i) = 1;
				curP2(i, i) = 1;
			}
			curPt1(0, 0) = curSeg.m_Track1.CenterPt.x;
			curPt1(1, 0) = curSeg.m_Track1.CenterPt.y;
			curPt1(2, 0) = curSeg.m_Track1.CenterPt.z;
			curPt2(0, 0) = curSeg.m_Track2.CenterPt.x;
			curPt2(1, 0) = curSeg.m_Track2.CenterPt.y;
			curPt2(2, 0) = curSeg.m_Track2.CenterPt.z;

			curTrkHgt1 = m_InitTrkHgt1;
			curTrkHgt2 = m_InitTrkHgt2;

			m_bInit = false;
		}
		else {
			POINT3D nPt = currentLayer->m_TrackSegList[currentLayer->m_TrackSegList.size() - 1].m_Track1.CenterPt;;
			POINT3D Pt = currentLayer->m_TrackSegList[currentLayer->m_TrackSegList.size() - 2].m_Track1.CenterPt;
			mode = GetDirectionMode(nPt.x, nPt.y, Pt.x, Pt.y);
			//		curP1 = hatP1; curPt1 = hatPt1;
			//		curP2 = hatP2; curPt2 = hatPt2;

			curPt1(0, 0) = curSeg.m_Track1.CenterPt.x;
			curPt1(1, 0) = curSeg.m_Track1.CenterPt.y;
			curPt1(2, 0) = curSeg.m_Track1.CenterPt.z;
			curPt2(0, 0) = curSeg.m_Track2.CenterPt.x;
			curPt2(1, 0) = curSeg.m_Track2.CenterPt.y;
			curPt2(2, 0) = curSeg.m_Track2.CenterPt.z;

			curTrkHgt1 = nextTrkHgt1;
			curTrkHgt2 = nextTrkHgt2;

			//	printf("curPt1: %0.3f %0.3f %0.3f  curPt2: %0.3f %0.3f %0.3f angle: %0.3f\n",
			//		curPt1(0,0),curPt1(1,0),curPt1(2,0),curPt2(0,0),curPt2(1,0),curPt2(2,0),curSeg.m_Track1.ori);
		}

		//predict next track points
		KalmanPredict(mode, curSeg.m_Track1.ori, m_gLength, curPt1, curP1, predPt1, predP1);
		KalmanPredict(mode, curSeg.m_Track2.ori, m_gLength, curPt2, curP2, predPt2, predP2);

		//generate next track segment and then assign member points
		TrackSegment nextSeg;
		GenerateTrkSegAssignMemPts(nextSeg, curSeg, predPt1, predPt2, m_gLength, m_gWidth, mode);

		//stop condition (if polygon does not have enough points)
		if (nextSeg.m_Track1.MemPnts.size() < m_ThMinPts || nextSeg.m_Track2.MemPnts.size() < m_ThMinPts) {
			//increase track window length
			bool flg = false;
			for (int i = 0; i < 5; i++) {
				double gLength = m_gLength + (i + 1)*m_gLength;
				KalmanPredict(mode, curSeg.m_Track1.ori, gLength, curPt1, curP1, predPt1, predP1);
				KalmanPredict(mode, curSeg.m_Track2.ori, gLength, curPt2, curP2, predPt2, predP2);

				GenerateTrkSegAssignMemPts(nextSeg, curSeg, predPt1, predPt2, gLength, m_gWidth, mode);
				if (nextSeg.m_Track1.MemPnts.size() > m_ThMinPts && nextSeg.m_Track2.MemPnts.size() > m_ThMinPts) {
					flg = true;
					break;
				}
			}
			if (flg == false) {
				printf("# of points in Track Window is too small!!\n");
				break;
			}
		}

		//Compute GMM parameter and Clustering
		vector<PointXYZDouble*> HeadPts1, HeadPts2;
		ExtractTrkPts(nextSeg, curTrkHgt1, curTrkHgt2, HeadPts1, HeadPts2, nextTrkHgt1, nextTrkHgt2);

		//stop condition (if polygon does not have enough points)
		if (HeadPts1.size() < m_ThMinTrkPts || HeadPts2.size() < m_ThMinTrkPts) {
			//increase window length
			bool flg = false;
			for (int i = 0; i < 5; i++) {
				double gLength = m_gLength + (i + 1)*m_gLength;
				KalmanPredict(mode, curSeg.m_Track1.ori, gLength, curPt1, curP1, predPt1, predP1);
				KalmanPredict(mode, curSeg.m_Track2.ori, gLength, curPt2, curP2, predPt2, predP2);

				GenerateTrkSegAssignMemPts(nextSeg, curSeg, predPt1, predPt2, gLength, m_gWidth, mode);
				ExtractTrkPts(nextSeg, curTrkHgt1, curTrkHgt2, HeadPts1, HeadPts2, nextTrkHgt1, nextTrkHgt2);

				if (HeadPts1.size() > m_ThMinTrkPts && HeadPts2.size() > m_ThMinTrkPts) {
					flg = true;
					break;
				}
			}
			if (flg == false) {
				printf("# of head points in Window\n");
				break;
			}
		}

		//visualization
		//	for(int i=0;i<HeadPts1.size();i++)
		//		HeadPts1[i]->ClassCode = 3;
		//	for(int i=0;i<HeadPts2.size();i++)
		//		HeadPts2[i]->ClassCode = 3;

		//variables 
		Line3D tmpLn1, tmpLn2, tmpLn3;
		double sigma1, sigma2;
		vector<PointXYZDouble*> supportPts1, supportPts2;
		vector<PointXYZDouble*> obsHeadPts1, obsHeadPts2;
		Line3D guideLn1 = curSeg.m_Track1.MemLn;
		Line3D guideLn2 = curSeg.m_Track2.MemLn;
		vector<Line3D> LnList1, LnList2;

		SegmentTrackPtsByMDL(mode, HeadPts1, LnList1, curSeg.m_Track1.ori);
		SegmentTrackPtsByMDL(mode, HeadPts2, LnList2, curSeg.m_Track2.ori);

		//Check turnout(or multi tracks) Region
		if (LnList1.size() > 1)
			nextSeg.m_Track1.bTurnOut = true;
		else
			nextSeg.m_Track1.bTurnOut = false;

		if (LnList2.size() > 1)
			nextSeg.m_Track2.bTurnOut = true;
		else
			nextSeg.m_Track2.bTurnOut = false;

		///If Turnout (or Anomaly) region is found 
		if (nextSeg.m_Track1.bTurnOut == true || nextSeg.m_Track2.bTurnOut == true) {
			//determine observation(supportPts1 and supportPts2)of rail track at turnout region
			DetermineObservation(TrackDirection, trkStk, guideLn1, guideLn2, LnList1, LnList2, mode, obsHeadPts1, obsHeadPts2,
				nextSeg.m_Track1.slMemPnts, nextSeg.m_Track1.srMemPnts, nextSeg.m_Track2.slMemPnts, nextSeg.m_Track2.srMemPnts);

			LineExtractionByRansac(obsHeadPts1, m_ThRansac, tmpLn1, sigma1, supportPts1);
			LineExtractionByRansac(obsHeadPts2, m_ThRansac, tmpLn2, sigma2, supportPts2);
			tmpLn1.SetEndByReferencePt(curSeg.m_Track1.CenterPt);
			tmpLn2.SetEndByReferencePt(curSeg.m_Track2.CenterPt);
			//	tmpLn1.SetEndingByMode(mode);
			//	tmpLn2.SetEndingByMode(mode);

			if (nextSeg.m_Track1.slMemPnts.size() != 0)
				nextSeg.m_Track1.bLeftTag = 1;
			if (nextSeg.m_Track1.srMemPnts.size() != 0)
				nextSeg.m_Track1.bRightTag = 1;
			if (nextSeg.m_Track2.slMemPnts.size() != 0)
				nextSeg.m_Track2.bLeftTag = 1;
			if (nextSeg.m_Track2.srMemPnts.size() != 0)
				nextSeg.m_Track2.bRightTag = 1;

			nextSeg.m_Track1.MemPnts.clear();
			nextSeg.m_Track2.MemPnts.clear();
			nextSeg.m_Track1.bTurnOut = true;
			nextSeg.m_Track2.bTurnOut = true;

			trkStk.trkSegs.push_back(nextSeg);
		}
		else {		//if the regions is not turn out region
			LineExtractionByRansac(HeadPts1, m_ThRansac, tmpLn1, sigma1, supportPts1);
			LineExtractionByRansac(HeadPts2, m_ThRansac, tmpLn2, sigma2, supportPts2);

			if (trkStk.trkSegs.size() > 2)
				m_TrkStks.push_back(trkStk);
			trkStk.trkSegs.clear();
		}

		//determine coordinate of observation for Kalman filter
		POINT3D mPt1(0, 0, 0);
		POINT3D mPt2(0, 0, 0);
		for (int i = 0; i < supportPts1.size(); i++) {
			mPt1.x = mPt1.x + supportPts1[i]->x;
			mPt1.y = mPt1.y + supportPts1[i]->y;
			mPt1.z = mPt1.z + supportPts1[i]->z;
		}
		mPt1 = mPt1 / supportPts1.size();

		for (int i = 0; i < supportPts2.size(); i++) {
			mPt2.x = mPt2.x + supportPts2[i]->x;
			mPt2.y = mPt2.y + supportPts2[i]->y;
			mPt2.z = mPt2.z + supportPts2[i]->z;
		}
		mPt2 = mPt2 / supportPts2.size();

		sort(supportPts1.begin(), supportPts1.end(), HigherZValue);
		sort(supportPts2.begin(), supportPts2.end(), HigherZValue);

		if (supportPts1.size() == 0 || supportPts2.size() == 0) {
			printf("no points\n");
			break;
		}

		int HgtIdx1 = (int)(supportPts1.size()*0.15);
		int HgtIdx2 = (int)(supportPts2.size()*0.15);

		obsPt1(0, 0) = mPt1.x; obsPt1(1, 0) = mPt1.y; obsPt1(2, 0) = supportPts1[HgtIdx1]->z;
		obsPt2(0, 0) = mPt2.x; obsPt2(1, 0) = mPt2.y; obsPt2(2, 0) = supportPts2[HgtIdx2]->z;

		KalmanUpdate(predPt1, predP1, obsPt1, sigma1, hatPt1, hatP1);
		KalmanUpdate(predPt2, predP2, obsPt2, sigma2, hatPt2, hatP2);

		tmpLn1.BeginPt.SetPoint3D(curPt1(0, 0), curPt1(1, 0), curPt1(2, 0));
		tmpLn1.EndPt.SetPoint3D(hatPt1(0, 0), hatPt1(1, 0), hatPt1(2, 0));
		tmpLn2.BeginPt.SetPoint3D(curPt2(0, 0), curPt2(1, 0), curPt2(2, 0));
		tmpLn2.EndPt.SetPoint3D(hatPt2(0, 0), hatPt2(1, 0), hatPt2(2, 0));
		nextTrkHgt1 = hatPt1(2, 0);
		nextTrkHgt2 = hatPt2(2, 0);
		tmpLn1.GetLineParameter();
		tmpLn2.GetLineParameter();
		tmpLn1.GetAngle();
		tmpLn2.GetAngle();
		tmpLn1.SetEndByReferencePt(curSeg.m_Track1.CenterPt);
		tmpLn2.SetEndByReferencePt(curSeg.m_Track2.CenterPt);
		//	tmpLn1.SetEndingByMode(mode);
		//	tmpLn2.SetEndingByMode(mode);
		double _LnDist = m_TrkGuage;
		int mode1 = GetDirectionMode(tmpLn1.EndPt.x, tmpLn1.EndPt.y, tmpLn1.BeginPt.x, tmpLn1.BeginPt.y);
		int mode2 = GetDirectionMode(tmpLn2.EndPt.x, tmpLn2.EndPt.y, tmpLn2.BeginPt.x, tmpLn2.BeginPt.y);
		//	AdjustTwoLines(tmpLn1, tmpLn2, mode1, mode2, mode,_LnDist);
		AdjustTwoLinesWithObservation(curSeg.m_Track1.CenterPt, curSeg.m_Track2.CenterPt, tmpLn1, tmpLn2, supportPts1, supportPts2, mode1, mode2, mode, _LnDist);

		tmpLn1.BeginPt.SetPoint3D(curSeg.m_Track1.CenterPt.x, curSeg.m_Track1.CenterPt.y, curSeg.m_Track1.CenterPt.z);
		tmpLn2.BeginPt.SetPoint3D(curSeg.m_Track2.CenterPt.x, curSeg.m_Track2.CenterPt.y, curSeg.m_Track2.CenterPt.z);
		tmpLn3.BeginPt = (tmpLn1.BeginPt + tmpLn2.BeginPt) / 2;
		tmpLn3.EndPt = (tmpLn1.EndPt + tmpLn2.EndPt) / 2;
		tmpLn1.GetLineParameter();
		tmpLn2.GetLineParameter();
		tmpLn3.GetLineParameter();
		currentLayer->m_RailVec1.push_back(tmpLn1.EndPt);
		currentLayer->m_RailVec2.push_back(tmpLn2.EndPt);
		currentLayer->m_RailVecCenter.push_back(tmpLn3.EndPt);

		//Distance from start point
		double fdist = DistanceBetweenPoints2D(tmpLn3.BeginPt, tmpLn3.EndPt);
		m_distFromSp = m_distFromSp + fdist;

		//update inital model vector
		nextSeg.m_Track1.ori = tmpLn1.Ang;
		nextSeg.m_Track1.CenterPt.SetPoint3D(tmpLn1.EndPt.x, tmpLn1.EndPt.y, tmpLn1.EndPt.z);
		nextSeg.m_Track1.MemLn = tmpLn1;
		mode = GetDirectionMode(tmpLn1.EndPt.x, tmpLn1.EndPt.y, tmpLn1.BeginPt.x, tmpLn1.BeginPt.y);
		nextSeg.m_Track1.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
		nextSeg.m_Track2.ori = tmpLn2.Ang;
		nextSeg.m_Track2.CenterPt.SetPoint3D(tmpLn2.EndPt.x, tmpLn2.EndPt.y, tmpLn2.EndPt.z);
		nextSeg.m_Track2.MemLn = tmpLn2;
		mode = GetDirectionMode(tmpLn2.EndPt.x, tmpLn2.EndPt.y, tmpLn2.BeginPt.x, tmpLn2.BeginPt.y);
		nextSeg.m_Track2.GenerateVtxUsingCenterAndAngle(m_gLength, m_gWidth, mode);
		nextSeg.GetCenterOfTwoTracks();
		nextSeg.GetDistanceBetweenTwoCenter();
		nextSeg.distFromSp = m_distFromSp;

		//Stop condition (if defference between calcuaged track guage and defined track guage is larger than XX)
		if (fabs(nextSeg.m_D - m_TrkGuage) > 0.15) {
			printf("Track Guage difference: %0.3f\n", fabs(nextSeg.m_D - m_TrkGuage));
			break;
		}

		//Stop condition 
		double dist = DistanceBetweenPoints2D(curSeg.m_Track1.CenterPt, nextSeg.m_Track1.CenterPt);
		if (dist < 0.3) {
			printf("Track End by trajectory\n");
			break;
		}
		//Loop region
		//		double aveHgtOfGround = (aveHgtOfGround1+aveHgtOfGround2)/2;
		//		double aveHgtOfTrack = (nextTrkHgt1+nextTrkHgt2)/2;
		//		Line3D refLn = tmpLn3;
		//		refLn.GetAngle();
		//	DetectLoopCross(nextSeg,rgnPts,aveHgtOfGround,aveHgtOfTrack,refLn,m_LoopCross);
		//		DetectLoopCrossDemo(nextSeg,rgnPts,aveHgtOfGround,aveHgtOfTrack,refLn,m_LoopCross);


		//set Z value of bounding box
		for (int i = 0; i < nextSeg.m_Track1.vtx.size(); i++)
			nextSeg.m_Track1.vtx[i].z = nextSeg.m_Track1.MemLn.BeginPt.z;
		for (int i = 0; i < nextSeg.m_Track2.vtx.size(); i++)
			nextSeg.m_Track2.vtx[i].z = nextSeg.m_Track2.MemLn.BeginPt.z;

		//Visualization (updated BNDs)
		if (nextSeg.m_Track1.bTurnOut == false) {
			vector<Line3D> tmpUpdateLns;
			GetLnsFromVtx(nextSeg.m_Track1.vtx, tmpUpdateLns);
			for (int i = 0; i < tmpUpdateLns.size(); i++)
				currentLayer->m_updateBnd.push_back(tmpUpdateLns[i]);
		}
		else {
			vector<Line3D> tmpUpdateLns;
			GetLnsFromVtx(nextSeg.m_Track1.vtx, tmpUpdateLns);
			for (int i = 0; i < tmpUpdateLns.size(); i++)
				currentLayer->m_updateBnd_turnout.push_back(tmpUpdateLns[i]);
		}

		if (nextSeg.m_Track2.bTurnOut == false) {
			vector<Line3D> tmpUpdateLns;
			GetLnsFromVtx(nextSeg.m_Track2.vtx, tmpUpdateLns);
			for (int i = 0; i < tmpUpdateLns.size(); i++)
				currentLayer->m_updateBnd.push_back(tmpUpdateLns[i]);
		}
		else {
			vector<Line3D> tmpUpdateLns;
			GetLnsFromVtx(nextSeg.m_Track2.vtx, tmpUpdateLns);
			for (int i = 0; i < tmpUpdateLns.size(); i++)
				currentLayer->m_updateBnd_turnout.push_back(tmpUpdateLns[i]);
		}

		iter++;

		//m_TrackSegList.push_back(nextSeg);
		currentLayer->m_TrackSegList.push_back(nextSeg);
		curSeg.DuplicatePolyPair(nextSeg);

		POINT3D nPt = nextSeg.m_Track1.MemLn.EndPt;
		POINT3D Pt = nextSeg.m_Track1.MemLn.BeginPt;
		int d_mode = GetDirectionMode(nPt.x, nPt.y, Pt.x, Pt.y);

		double angel = GetAngle(nPt.x, nPt.y, Pt.x, Pt.y);
		printf("centerPt: %f,%f,%f -- mode: %d -- angel: %f\n", nextSeg.m_Center.x, nextSeg.m_Center.y, nextSeg.m_Center.z, d_mode, angel);


		double d_x = nPt.x - Pt.x;
		double d_y = nPt.y - Pt.y;

		abs(d_x);

		if (abs(d_x) > abs(d_y))
		{
			double total = m_BoundMax.x - m_BoundMin.x;
			if (d_x > 0)
			{
				progressCallback(nextSeg.m_Center.x - m_BoundMin.x, total);
			}
			else
			{
				progressCallback(m_BoundMax.x - nextSeg.m_Center.x, total);
			}
		}
		else
		{
			double total = m_BoundMax.y - m_BoundMin.y;
			if (d_x > 0)
			{
				progressCallback(nextSeg.m_Center.y - m_BoundMin.y, total);
			}
			else
			{
				progressCallback(m_BoundMax.y - nextSeg.m_Center.y, total);
			}
		}

		if (mode == 1 || mode == 4)
		{
			//m_BoundMax.x
			double total = m_BoundMax.x - m_BoundMin.x;
			progressCallback(nextSeg.m_Center.x - m_BoundMin.x, total);
		}

		

		//real time visualization
		//if (iter % 5 == 0) {
			updateView(1);
		//}
	}

	//Define Templetes
	DefineTemplates();

	//Template Matching ///////////////////
	if (mainTrkflg == true) {
		for (int i = 0; i<m_TrkStks.size(); i++) {
			if (m_TrkStks[i].trkSegs.size() <= 4)
				continue;

			int b1 = m_TrkStks[i].trkSegs[0].m_Track2.bLeftTag;
			int b2 = m_TrkStks[i].trkSegs[0].m_Track2.bRightTag;
			int b3 = m_TrkStks[i].trkSegs[0].m_Track1.bLeftTag;
			int b4 = m_TrkStks[i].trkSegs[0].m_Track1.bRightTag;

			int b5 = m_TrkStks[i].trkSegs[1].m_Track2.bLeftTag;
			int b6 = m_TrkStks[i].trkSegs[1].m_Track2.bRightTag;
			int b7 = m_TrkStks[i].trkSegs[1].m_Track1.bLeftTag;
			int b8 = m_TrkStks[i].trkSegs[1].m_Track1.bRightTag;

			int b9 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 2].m_Track2.bLeftTag;
			int b10 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 2].m_Track2.bRightTag;
			int b11 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 2].m_Track1.bLeftTag;
			int b12 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 2].m_Track1.bRightTag;

			int b13 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 1].m_Track2.bLeftTag;
			int b14 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 1].m_Track2.bRightTag;
			int b15 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 1].m_Track1.bLeftTag;
			int b16 = m_TrkStks[i].trkSegs[m_TrkStks[i].trkSegs.size() - 1].m_Track1.bRightTag;

			m_TrkStks[i].trkBinary.SetBinaryString(b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16);

			BinaryString preBinary;
			if (i == 0) {
				preBinary.Type = -1;
				preBinary.LabelPosTurnout = 1;
			}
			else
				preBinary = m_TrkStks[i - 1].trkBinary;

			printf("PosTurnout: %d, Direction: %d, TurnoutType: %d\n", preBinary.LabelPosTurnout, preBinary.LabelDirection, preBinary.LabelTurnoutType);

			RecognitionTurnoutType(m_Tmplts, preBinary, m_TrkStks[i].trkBinary);

			/*	int preType;
			int tmpltType = 0;
			if(i==0)
			tmpltType = 0;
			else{
			preType = m_TrkStks[i-1].trkBinary.Type;
			if(preType == 0 || preType == 1 || preType == 2 || preType == 3 || preType == 4 ||
			preType == 5 ||	preType == 12 || preType == 13 || preType == 16 || preType == 17)
			tmpltType = 1;
			else
			tmpltType = 0;
			}

			//	TemplateMatching(m_Tmplts,m_TrkStks[i].trkBinary);
			if(tmpltType == 0)
			TemplateMatching(m_TmpltBegin,m_TrkStks[i].trkBinary);
			else
			TemplateMatching(m_TmpltEnd,m_TrkStks[i].trkBinary);*/
		}

		//validation of template matching
		//	ValidateTurnoutType(m_TrkStks);

		//	PrintLines(m_updateBnd_turnout);
		//detect Toes
		for (int i = 0; i<m_TrkStks.size(); i++) {
			if (m_TrkStks[i].trkSegs.size() <= 4)
				continue;

			if (m_TrkStks[i].trkBinary.Score < 0.9)
				continue;

			ToePt toe;
			bool flg = DetectToePnts(m_TrkStks[i].trkBinary.Type, m_TrkStks[i].trkSegs, toe);
			if (flg == true) {
				m_ToePts.push_back(toe);
				currentLayer->m_ToePts.push_back(toe);
			}

		}

		//Detect switch box
		DetectSwitchBox(m_ToePts, m_SwitchBoxes);
	}

	//Print loop cross
	/*	printf("Loop Cross\n");
	for(int i=0;i<m_LoopCross.size();i++)
	printf("%0.3f\n",m_LoopCross[i].distFromSp);*/

	//	ValidateLoopCross(m_LoopCross);
	/*	printf("After LoopCross\n");
	for(int i=0;i<m_LoopCross.size();i++){
	printf("%0.2f\n",m_LoopCross[i].distFromSp);
	m_visulLoopCross.push_back(m_LoopCross[i]);
	}*/

}

/**
 * @fn	void RailwayExtraction::ExtractTrkPts(TrackSegment nSeg, double curTrkHgt1, double curTrkHgt2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2, double &nextTrkHgt1, double &nextTrkHgt2)
 *
 * @brief	Extracts the tract points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	nSeg	   	The next track segment.
 * @param 		  	curTrkHgt1 	The height of the current track 1.
 * @param 		  	curTrkHgt2 	The height of the current track 2.
 * @param [in,out]	HeadPts1   	[in,out] If non-null, the first head points.
 * @param [in,out]	HeadPts2   	[in,out] If non-null, the second head points.
 * @param [in,out]	nextTrkHgt1	The first next track height.
 * @param [in,out]	nextTrkHgt2	The second next track height.
 */

void RailwayExtraction::ExtractTrkPts(TrackSegment nSeg, double curTrkHgt1, double curTrkHgt2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2, double &nextTrkHgt1, double &nextTrkHgt2)
{
	vector<PointXYZDouble*> tmpPts1, tmpPts2;
	double ThHgtOfTrack = 0.1;

	for (int i = 0; i<nSeg.m_Track1.MemPnts.size(); i++) {
		double h = nSeg.m_Track1.MemPnts[i]->z;
		if (h > curTrkHgt1 + ThHgtOfTrack)
			continue;
		if (h < curTrkHgt1 - 2.5*ThHgtOfTrack)
			continue;
		tmpPts1.push_back(nSeg.m_Track1.MemPnts[i]);
	}

	for (int i = 0; i<nSeg.m_Track2.MemPnts.size(); i++) {
		double h = nSeg.m_Track2.MemPnts[i]->z;
		if (h > curTrkHgt2 + ThHgtOfTrack)
			continue;
		if (h < curTrkHgt2 - 2.5*ThHgtOfTrack)
			continue;
		tmpPts2.push_back(nSeg.m_Track2.MemPnts[i]);
	}

	//Compute GMM parameter
	vector<GMMPara> paras1, paras2;
	ComputeGMMfeature(tmpPts1, paras1, 3);
	ComputeGMMfeature(tmpPts2, paras2, 3);

	//Clustering by GMM parameter
	vector<vector<PointXYZDouble*>> RailPts1, RailPts2;
	ClusteringByGMM(tmpPts1, paras1, RailPts1);
	ClusteringByGMM(tmpPts2, paras2, RailPts2);

	vector<PointXYZDouble*> tmpHeadPts1, tmpHeadPts2;
	vector<PointXYZDouble*> tmpGrdPts1, tmpGrdPts2;

	for (int i = 0; i < RailPts1[RailPts1.size() - 1].size(); i++) {
	//	if(RailPts1[RailPts1.size()-1][i]->Intensity > m_ThMinIntensityForTrack
	//		&& RailPts1[RailPts1.size()-1][i]->Intensity < m_ThMaxIntensityForTrack)
			tmpHeadPts1.push_back(RailPts1[RailPts1.size() - 1][i]);
	}
	for (int i = 0; i < RailPts1[RailPts1.size() - 2].size(); i++) {
	//	if (RailPts1[RailPts1.size() - 2][i]->Intensity > m_ThMinIntensityForTrack
	//		&& RailPts1[RailPts1.size() - 2][i]->Intensity < m_ThMaxIntensityForTrack)
			tmpHeadPts1.push_back(RailPts1[RailPts1.size() - 2][i]);
	}
		
	for (int i = 0; i < RailPts2[RailPts2.size() - 1].size(); i++) {
	//	if (RailPts2[RailPts1.size() - 1][i]->Intensity > m_ThMinIntensityForTrack
	//		&& RailPts2[RailPts1.size() - 1][i]->Intensity < m_ThMaxIntensityForTrack)
			tmpHeadPts2.push_back(RailPts2[RailPts2.size() - 1][i]);
	}	
	for (int i = 0; i < RailPts2[RailPts2.size() - 2].size(); i++) {
	//	if (RailPts2[RailPts1.size() - 2][i]->Intensity > m_ThMinIntensityForTrack
	//		&& RailPts2[RailPts1.size() - 2][i]->Intensity < m_ThMaxIntensityForTrack)
			tmpHeadPts2.push_back(RailPts2[RailPts2.size() - 2][i]);
	}
		
	tmpGrdPts1 = RailPts1[0];
	tmpGrdPts2 = RailPts2[0];

	//	for(int i=0;i<tmpHeadPts1.size();i++)
	//		tmpHeadPts1[i]->ClassCode = 10;
	//	for(int i=0;i<tmpHeadPts2.size();i++)
	//		tmpHeadPts2[i]->ClassCode = 10;
	//	for(int i=0;i<tmpGrdPts1.size();i++)
	//		tmpGrdPts1[i]->ClassCode = 11;
	//	for(int i=0;i<tmpGrdPts2.size();i++)
	//		tmpGrdPts2[i]->ClassCode = 11;

	Line3D ln1, ln2;
	LineExtractionByRansac(tmpHeadPts1, 0.04, ln1);
	LineExtractionByRansac(tmpHeadPts2, 0.04, ln2);
	ln1.GetUnitVec();
	ln2.GetUnitVec();
	ln1.GetLineParameter();
	ln2.GetLineParameter();

	//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln1.BeginPt.x,ln1.BeginPt.y,ln1.BeginPt.z,ln1.EndPt.x,ln1.EndPt.y,ln1.EndPt.z);
	//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln2.BeginPt.x,ln2.BeginPt.y,ln2.BeginPt.z,ln2.EndPt.x,ln2.EndPt.y,ln2.EndPt.z);

	for (int i = 0; i<tmpHeadPts1.size(); i++) {
		double fdist3 = DistanceBetweenPointAndLine3DHgt(tmpHeadPts1[i], ln1);
		if (fdist3 < 0.04)
			HeadPts1.push_back(tmpHeadPts1[i]);
	}
	for (int i = 0; i<tmpHeadPts2.size(); i++) {
		double fdist3 = DistanceBetweenPointAndLine3DHgt(tmpHeadPts2[i], ln2);
		if (fdist3 < 0.04)
			HeadPts2.push_back(tmpHeadPts2[i]);
	}

	for (int i = 0; i<HeadPts1.size(); i++)
		HeadPts1[i]->ClassCode = 3;
	for (int i = 0; i<HeadPts2.size(); i++)
		HeadPts2[i]->ClassCode = 3;

	if (HeadPts1.size() == 0 || HeadPts2.size() == 0) {
		nextTrkHgt1 = curTrkHgt1;
		nextTrkHgt2 = curTrkHgt2;
		printf("no head points in ExtractTrkPts!!\n");
		return;
	}

	//compute height of initial track points
	sort(HeadPts1.begin(), HeadPts1.end(), HigherZValue);
	sort(HeadPts2.begin(), HeadPts2.end(), HigherZValue);

	int HgtIdx1 = (int)(HeadPts1.size()*0.15);
	int HgtIdx2 = (int)(HeadPts2.size()*0.15);

	nextTrkHgt1 = HeadPts1[HgtIdx1]->z;
	nextTrkHgt2 = HeadPts2[HgtIdx2]->z;
}

/**
 * @fn	void RailwayExtraction::ComputeGMMParaAndCluster(TrackSegment nextSeg, vector<GMMPara> &nextParas1, vector<GMMPara> &nextParas2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2)
 *
 * @brief	Calculates the gmm para and cluster points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	nextSeg   	The next track segment.
 * @param [in,out]	nextParas1	The GMM parameters for first track.
 * @param [in,out]	nextParas2	The GMM parameters for second track.
 * @param [in,out]	HeadPts1  	[in,out] If non-null, the head points belonging to first track window.
 * @param [in,out]	HeadPts2  	[in,out] If non-null, the head points belonging to second track window.
 */

void RailwayExtraction::ComputeGMMParaAndCluster(TrackSegment nextSeg, vector<GMMPara> &nextParas1, vector<GMMPara> &nextParas2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2)
{
	//Compute GMM parameter
	vector<PointXYZDouble*> tmpPnts1 = nextSeg.m_Track1.MemPnts;
	vector<PointXYZDouble*> tmpPnts2 = nextSeg.m_Track2.MemPnts;
	ComputeGMMfeature(tmpPnts1, nextParas1, 3);
	ComputeGMMfeature(tmpPnts2, nextParas2, 3);

	//	printf("%0.3f,%0.3f,%0.3f\n",nextParas1[0].mu,nextParas1[1].mu, nextParas1[2].mu);
	//	printf("%0.3f,%0.3f,%0.3f\n",nextParas2[0].mu,nextParas2[1].mu, nextParas2[2].mu);
	//Clustering by GMM Parameter
	vector<vector<PointXYZDouble*>> RailPts1, RailPts2;
	ClusteringByGMM(tmpPnts1, nextParas1, RailPts1);
	ClusteringByGMM(tmpPnts2, nextParas2, RailPts2);

	HeadPts1 = RailPts1[RailPts1.size() - 1];
	HeadPts2 = RailPts2[RailPts2.size() - 1];
}

/**
 * @fn	void RailwayExtraction::DeleteSegPntsCmpWithPreHgt(TrackSegment &nextSeg, double aveHgtOfTrack1, double aveHgtOfTrack2, double aveHgtOfGround1, double aveHgtOfGround2, double ThHgtOfTrack, double ThHgtOfGround)
 *
 * @brief	Deletes track segment points with height
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	nextSeg		   	The next track segment.
 * @param 		  	aveHgtOfTrack1 	The average track height of first track.
 * @param 		  	aveHgtOfTrack2 	The average track height of second track.
 * @param 		  	aveHgtOfGround1	The average ground height of first track.
 * @param 		  	aveHgtOfGround2	The average ground height of second track.
 * @param 		  	ThHgtOfTrack   	The threshold for track height.
 * @param 		  	ThHgtOfGround  	The threshold for ground height.
 */

void RailwayExtraction::DeleteSegPntsCmpWithPreHgt(TrackSegment &nextSeg, double aveHgtOfTrack1, double aveHgtOfTrack2, double aveHgtOfGround1, double aveHgtOfGround2, double ThHgtOfTrack, double ThHgtOfGround)
{
	//delete higher points compared with previous track height
	for (int i = 0; i<nextSeg.m_Track1.MemPnts.size();) {
		if (nextSeg.m_Track1.MemPnts[i]->z > aveHgtOfTrack1 + ThHgtOfTrack)
			nextSeg.m_Track1.MemPnts.erase(nextSeg.m_Track1.MemPnts.begin() + i);
		else
			i++;
	}
	for (int i = 0; i<nextSeg.m_Track2.MemPnts.size();) {
		if (nextSeg.m_Track2.MemPnts[i]->z > aveHgtOfTrack2 + ThHgtOfTrack)
			nextSeg.m_Track2.MemPnts.erase(nextSeg.m_Track2.MemPnts.begin() + i);
		else
			i++;
	}
	//delet lower points compared with previous ground height
	for (int i = 0; i<nextSeg.m_Track1.MemPnts.size();) {
		if (nextSeg.m_Track1.MemPnts[i]->z < aveHgtOfGround1 - ThHgtOfGround)
			nextSeg.m_Track1.MemPnts.erase(nextSeg.m_Track1.MemPnts.begin() + i);
		else
			i++;
	}
	for (int i = 0; i<nextSeg.m_Track2.MemPnts.size();) {
		if (nextSeg.m_Track2.MemPnts[i]->z < aveHgtOfGround2 - ThHgtOfGround)
			nextSeg.m_Track2.MemPnts.erase(nextSeg.m_Track2.MemPnts.begin() + i);
		else
			i++;
	}
}

/**
 * @fn	void RailwayExtraction::GenerateTrkSegAssignMemPts(TrackSegment &nextSeg, TrackSegment curSeg, Matrix<double> predPt1, Matrix<double> predPt2, double Length, double Width, int mode)
 *
 * @brief	Assign points to track segment 
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	nextSeg	The next track segment.
 * @param 		  	curSeg 	The current track segment.
 * @param 		  	predPt1	The predicted center coordinate of first track window.
 * @param 		  	predPt2	The predicted center coordinate of second track window.
 * @param 		  	Length 	The length of track window.
 * @param 		  	Width  	The width of track window.
 * @param 		  	mode   	The mode.
 */

void RailwayExtraction::GenerateTrkSegAssignMemPts(TrackSegment &nextSeg, TrackSegment curSeg, Matrix<double> predPt1, Matrix<double> predPt2, double Length, double Width, int mode)
{
	nextSeg.m_Track1.ori = curSeg.m_Track1.ori;
	nextSeg.m_Track2.ori = curSeg.m_Track2.ori;
	nextSeg.m_Track1.CenterPt.SetPoint3D(predPt1(0, 0), predPt1(1, 0), predPt1(2, 0));
	nextSeg.m_Track1.GenerateVtxUsingCenterAndAngle(Length, Width, mode);
	nextSeg.m_Track2.CenterPt.SetPoint3D(predPt2(0, 0), predPt2(1, 0), predPt2(2, 0));
	nextSeg.m_Track2.GenerateVtxUsingCenterAndAngle(Length, Width, mode);
	nextSeg.GetCenterOfTwoTracks();
	nextSeg.GetDistanceBetweenTwoCenter();

	//Assign member points
	vector<int> neighborIdx = GetNeighborIdx(nextSeg.m_Center.x, nextSeg.m_Center.y, m_NumX, m_NumY, m_GridSize, 5);
	vector<PointXYZDouble*> rgnPts;
	for (int i = 0; i<neighborIdx.size(); i++) {
		if (neighborIdx[i]<0 || neighborIdx[i] >= m_NumGrid)
			continue;
		for (int j = 0; j<m_pGrid[neighborIdx[i]].MemPnts.size(); j++)
			rgnPts.push_back(m_pGrid[neighborIdx[i]].MemPnts[j]);
	}
	nextSeg.m_Track1.SetTrackWindowMemPnts(rgnPts);
	nextSeg.m_Track2.SetTrackWindowMemPnts(rgnPts);
}

/**
 * @fn	void RailwayExtraction::DetermineObservation(int TrackMode, TrackStack trkStk, Line3D guideln1, Line3D guideln2, vector<Line3D> lnList1, vector<Line3D> lnList2, int lnMode, vector<PointXYZDouble*> &obsPts1, vector<PointXYZDouble*> &obsPts2, vector<PointXYZDouble*> &LeftPts1, vector<PointXYZDouble*> &RightPts1, vector<PointXYZDouble*> &LeftPts2, vector<PointXYZDouble*> &RightPts2)
 *
 * @brief	Determine observations
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	TrackMode	The track mode.
 * @param 		  	trkStk   	The track stack.
 * @param 		  	guideln1 	The guide line for first track.
 * @param 		  	guideln2 	The guide line for second track.
 * @param 		  	lnList1  	The line list for first track.
 * @param 		  	lnList2  	The line list for second track.
 * @param 		  	lnMode   	The line mode.
 * @param [in,out]	obsPts1  	[in,out] If non-null, the observations for first track.
 * @param [in,out]	obsPts2  	[in,out] If non-null, the observations for second track.
 * @param [in,out]	LeftPts1 	[in,out] If non-null, the left points in first track.
 * @param [in,out]	RightPts1	[in,out] If non-null, the right points in first track.
 * @param [in,out]	LeftPts2 	[in,out] If non-null, the left points in second track.
 * @param [in,out]	RightPts2	[in,out] If non-null, the right points in second track.
 */

void RailwayExtraction::DetermineObservation(int TrackMode, TrackStack trkStk, Line3D guideln1, Line3D guideln2,
	vector<Line3D> lnList1, vector<Line3D> lnList2, int lnMode, vector<PointXYZDouble*> &obsPts1, vector<PointXYZDouble*> &obsPts2,
	vector<PointXYZDouble*> &LeftPts1, vector<PointXYZDouble*> &RightPts1, vector<PointXYZDouble*> &LeftPts2, vector<PointXYZDouble*> &RightPts2)
{
	double _LnDist = m_TrkGuage + 0.02;
	int nStack = trkStk.trkSegs.size();

	obsPts1.clear();
	obsPts2.clear();
	LeftPts1.clear();
	LeftPts2.clear();
	RightPts1.clear();
	RightPts2.clear();

	Line3D obsLn1, obsLn2;
	if (lnList1.size() == 1 && lnList2.size() == 1) {
		obsPts1 = lnList1[0].MemPts;
		obsPts2 = lnList2[0].MemPts;
		return;
	}

	//sort lines according to direction
	POINT3D lvec1, lvec2;
	POINT3D rvec1, rvec2;
	vector<Line3D> Rails1, Rails2;
	vector<double> Vals1, Vals2;
	vector<double> tmpVals1, tmpVals2;
	lvec1 = guideln1.EndPt - guideln1.BeginPt;
	for (int i = 0; i<lnList1.size(); i++) {
		lvec2 = lnList1[i].CenterPt - guideln1.BeginPt;
		double fdist = DistanceBetweenPointAndLine2D(lnList1[i].CenterPt, guideln1);
		double discriminant = lvec1.x*lvec2.y - lvec1.y*lvec2.x;
		if (discriminant > 0)
			fdist = -1 * fdist;
		Vals1.push_back(fdist);
	}
	tmpVals1 = Vals1;
	sort(tmpVals1.begin(), tmpVals1.end(), less<double>());
	for (int i = 0; i<tmpVals1.size(); i++) {
		for (int j = 0; j<Vals1.size(); j++) {
			if (tmpVals1[i] == Vals1[j]) {
				Rails1.push_back(lnList1[j]);
				break;
			}
		}
	}

	rvec1 = guideln2.EndPt - guideln2.BeginPt;
	for (int i = 0; i<lnList2.size(); i++) {
		rvec2 = lnList2[i].CenterPt - guideln2.BeginPt;
		double fdist = DistanceBetweenPointAndLine2D(lnList2[i].CenterPt, guideln2);
		double discriminant = rvec1.x*rvec2.y - rvec1.y*rvec2.x;
		if (discriminant > 0)
			fdist = -1 * fdist;
		Vals2.push_back(fdist);
	}
	tmpVals2 = Vals2;
	sort(tmpVals2.begin(), tmpVals2.end(), less<double>());
	for (int i = 0; i<tmpVals2.size(); i++) {
		for (int j = 0; j<Vals2.size(); j++) {
			if (tmpVals2[i] == Vals2[j]) {
				Rails2.push_back(lnList2[j]);
				break;
			}
		}
	}

	int cIdx1, cIdx2;
	Vals1.clear();
	Vals2.clear();
	for (int i = 0; i<tmpVals1.size(); i++)
		Vals1.push_back(fabs(tmpVals1[i]));
	for (int i = 0; i<tmpVals2.size(); i++)
		Vals2.push_back(fabs(tmpVals2[i]));

	tmpVals1 = Vals1;
	tmpVals2 = Vals2;
	sort(tmpVals1.begin(), tmpVals1.end(), less<double>());
	sort(tmpVals2.begin(), tmpVals2.end(), less<double>());

	for (int i = 0; i<Vals1.size(); i++) {
		if (tmpVals1[0] == Vals1[i]) {
			cIdx1 = i;
			break;
		}
	}
	for (int i = 0; i<Vals2.size(); i++) {
		if (tmpVals2[0] == Vals2[i]) {
			cIdx2 = i;
			break;
		}
	}

	double fdist1, fdist2, fdist3;
	if (TrackMode == 0) {
		if (Rails1.size() == 1) {
			obsLn1 = Rails1[0];
			obsPts1 = Rails1[0].MemPts;
			if (Rails2.size() == 2) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				if (fdist1 < fdist2) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}
				else {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
			else if (Rails2.size() == 3) {
				//	obsLn2 = Rails2[1];
				//	obsPts2 = Rails2[1].MemPts;
				//	LeftPts2 = Rails2[0].MemPts;
				//	RightPts2 = Rails2[2].MemPts;

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[2].CenterPt, obsLn1));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist3 <fdist1 && fdist3 < fdist2) {
					obsLn2 = Rails2[2];
					obsPts2 = Rails2[2].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
		}
		else if (Rails1.size() == 2) {
			if (Rails2.size() == 1) {
				obsLn2 = Rails2[0];
				obsPts2 = Rails2[0].MemPts;

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				if (fdist1 < fdist2) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[1].MemPts;
				}
				else {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
			else if (Rails2.size() == 2) {
				if (nStack<2) {		//at start
					obsLn1 = Rails1[cIdx1];
					obsPts1 = Rails1[cIdx1].MemPts;
					if (cIdx1 == 0)
						RightPts1 = Rails1[1].MemPts;
					else
						LeftPts1 = Rails1[0].MemPts;
				}
				else {				//at end
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				if (fdist1 < fdist2) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}
				else {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
			else if (Rails2.size() == 3) {
				////////
				if (nStack<2) {		//at start
					obsLn1 = Rails1[cIdx1];
					obsPts1 = Rails1[cIdx1].MemPts;
					if (cIdx1 == 0)
						RightPts1 = Rails1[1].MemPts;
					else
						LeftPts1 = Rails1[0].MemPts;
				}
				else {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[2].CenterPt, obsLn1));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist3 <fdist1 && fdist3 < fdist2) {
					obsLn2 = Rails2[2];
					obsPts2 = Rails2[2].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
		}
		else if (Rails1.size() == 3) {
			if (nStack<2) {		//at start
				obsLn1 = Rails1[cIdx1];
				obsPts1 = Rails1[cIdx1].MemPts;
				if (cIdx1 == 0)
					RightPts1 = Rails1[2].MemPts;
				else if (cIdx1 == 1) {
					LeftPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else
					LeftPts1 = Rails1[0].MemPts;
			}
			else {
				obsLn1 = Rails1[2];
				obsPts1 = Rails1[2].MemPts;
				LeftPts1 = Rails1[0].MemPts;
			}

			if (Rails2.size() == 1) {
				obsLn2 = Rails2[0];
				obsPts2 = Rails2[0].MemPts;
			}
			else if (Rails2.size() == 2) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				if (fdist1 < fdist2) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}
				else {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
			else if (Rails2.size() == 3) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[2].CenterPt, obsLn1));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[2].MemPts;
				}
				else if (fdist3 <fdist1 && fdist3 < fdist2) {
					obsLn2 = Rails2[2];
					obsPts2 = Rails2[2].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
		}
	}
	else {
		if (Rails2.size() == 1) {
			obsLn2 = Rails2[0];
			obsPts2 = Rails2[0].MemPts;
			if (Rails1.size() == 2) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				if (fdist1 < fdist2) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[1].MemPts;
				}
				else {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
			else if (Rails1.size() == 3) {
				//	obsLn1 = Rails1[1];
				//	obsPts1 = Rails1[1].MemPts;
				//	LeftPts1 = Rails1[0].MemPts;
				//	RightPts1 = Rails1[2].MemPts;

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[2].CenterPt, obsLn2));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist3 < fdist1 && fdist3 <fdist2) {
					obsLn1 = Rails1[2];
					obsPts1 = Rails1[2].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
		}
		else if (Rails2.size() == 2) {
			if (Rails1.size() == 1) {
				obsLn1 = Rails1[0];
				obsPts1 = Rails1[0].MemPts;

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[0].CenterPt, obsLn1));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails2[1].CenterPt, obsLn1));
				if (fdist1 < fdist2) {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}
				else {
					obsLn2 = Rails2[1];
					obsPts2 = Rails2[1].MemPts;
					LeftPts2 = Rails2[0].MemPts;
				}
			}
			else if (Rails1.size() == 2) {
				if (nStack <= 2) {		//at start
					obsLn2 = Rails2[cIdx2];
					obsPts2 = Rails2[cIdx2].MemPts;
					if (cIdx2 == 0)
						RightPts2 = Rails2[1].MemPts;
					else
						LeftPts2 = Rails2[0].MemPts;
				}
				else {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				if (fdist1 < fdist2) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[1].MemPts;
				}
				else {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}

			}
			else if (Rails1.size() == 3) {
				if (nStack <= 2) {
					obsLn2 = Rails2[cIdx2];
					obsPts2 = Rails2[cIdx2].MemPts;
					if (cIdx2 == 0)
						RightPts2 = Rails2[1].MemPts;
					else
						LeftPts2 = Rails2[0].MemPts;
				}
				else {
					obsLn2 = Rails2[0];
					obsPts2 = Rails2[0].MemPts;
					RightPts2 = Rails2[1].MemPts;
				}

				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[2].CenterPt, obsLn2));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist3 < fdist1 && fdist3 <fdist2) {
					obsLn1 = Rails1[2];
					obsPts1 = Rails1[2].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
		}
		else if (Rails2.size() == 3) {
			//	if(nStack <= 2){
			obsLn2 = Rails2[cIdx2];
			obsPts2 = Rails2[cIdx2].MemPts;
			if (cIdx2 == 0)
				RightPts2 = Rails2[2].MemPts;
			else if (cIdx2 == 1) {
				LeftPts2 = Rails2[0].MemPts;
				RightPts2 = Rails2[2].MemPts;
			}
			else
				LeftPts2 = Rails2[0].MemPts;
			//	}
			//	else{
			//		obsLn2 = Rails2[0];
			//		obsPts2 = Rails2[0].MemPts;
			//		RightPts2 = Rails2[2].MemPts;
			//	}

			if (Rails1.size() == 1) {
				obsLn1 = Rails1[0];
				obsPts1 = Rails1[0].MemPts;
			}
			else if (Rails1.size() == 2) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				if (fdist1 < fdist2) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[1].MemPts;
				}
				else {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
			else if (Rails1.size() == 3) {
				fdist1 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[0].CenterPt, obsLn2));
				fdist2 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[1].CenterPt, obsLn2));
				fdist3 = fabs(_LnDist - DistanceBetweenPointAndLine2D(Rails1[2].CenterPt, obsLn2));
				if (fdist1 < fdist2 && fdist1 < fdist3) {
					obsLn1 = Rails1[0];
					obsPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist2 < fdist1 && fdist2 < fdist3) {
					obsLn1 = Rails1[1];
					obsPts1 = Rails1[1].MemPts;
					LeftPts1 = Rails1[0].MemPts;
					RightPts1 = Rails1[2].MemPts;
				}
				else if (fdist3 < fdist1 && fdist3 <fdist2) {
					obsLn1 = Rails1[2];
					obsPts1 = Rails1[2].MemPts;
					LeftPts1 = Rails1[0].MemPts;
				}
			}
		}
	}


	int mode1, mode2;
	mode1 = GetDirectionMode(obsLn1.EndPt.x, obsLn1.EndPt.y, obsLn1.BeginPt.x, obsLn1.BeginPt.y);
	mode2 = GetDirectionMode(obsLn2.EndPt.x, obsLn2.EndPt.y, obsLn2.BeginPt.x, obsLn2.BeginPt.y);
	AdjustTwoLines(obsLn1, obsLn2, mode1, mode2, lnMode, _LnDist);

	/*	for(int i=0; i<obsPts1.size();i++)
	obsPts1[i]->ClassCode = 5;
	for(int i=0; i<obsPts2.size();i++)
	obsPts2[i]->ClassCode = 5;
	for(int i=0; i<LeftPts1.size();i++)
	LeftPts1[i]->ClassCode = 7;
	for(int i=0; i<LeftPts2.size();i++)
	LeftPts2[i]->ClassCode = 7;
	for(int i=0; i<RightPts1.size();i++)
	RightPts1[i]->ClassCode = 8;
	for(int i=0; i<RightPts2.size();i++)
	RightPts2[i]->ClassCode = 8;*/
}

/**
 * @fn	void RailwayExtraction::SegmentTrackPtsByMDL(int mode, vector<PointXYZDouble*> Pts, vector<Line3D> &lnList, double ang)
 *
 * @brief	Segment track points by model selection 
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	mode  	The mode.
 * @param [in,out]	Pts   	If non-null, the input points.
 * @param [in,out]	lnList	List of lines.
 * @param 		  	ang   	The angle.
 */

void RailwayExtraction::SegmentTrackPtsByMDL(int mode, vector<PointXYZDouble*> Pts, vector<Line3D> &lnList, double ang)
{
	vector<PointXYZDouble*> tmpPts;
	vector<PointXYZDouble*> sptPnts1, sptPnts2, sptPnts3;
	vector<PointXYZDouble*> segPnts1, segPnts2, segPnts3;
	Line3D ln1, ln2, ln3;
	double sigma;
	int minNofPts = (int)50. / m_skip_rate;		//300
	double minLength = 0.2;	//0.2
	bool flg1 = false, flg2 = false, flg3 = false;

	//first line
	//	if(Pts.size()>minNofPts){
	//	LineExtractionByRansac(Pts,m_ThRansac,ln1,sigma,sptPnts1);
	LineExtractionWithFixedAngle(Pts, ang, m_ThRansac, ln1, sptPnts1);
	ln1.SetEndingByMode(mode);
	segPnts1 = sptPnts1;
	ln1.MemPts.clear();
	ln1.MemPts = segPnts1;
	flg1 = true;
	//	}
	//	else 
	//		return;

	tmpPts.clear();
	for (int i = 0; i<Pts.size(); i++) {
		bool flg = false;
		for (int j = 0; j<segPnts1.size(); j++) {
			if (Pts[i] == segPnts1[j]) {
				flg = true;
				break;
			}
		}
		if (flg == false)
			tmpPts.push_back(Pts[i]);
	}

	//Second line
	if (tmpPts.size()> minNofPts) {
		//	LineExtractionByRansac(tmpPts,m_ThRansac,ln2,sigma,sptPnts2);
		LineExtractionWithFixedAngle(tmpPts, ang, m_ThRansac, ln2, sptPnts2);
		SegmentPoints(sptPnts2, segPnts2, m_ThDistGrouping);
		if (segPnts2.size()> minNofPts) {
			//		LineExtractionByRansac(segPnts2,m_ThRansac,ln2);
			LineExtractionWithFixedAngle(segPnts2, ang, m_ThRansac, ln2, sptPnts2);
			ln2.SetEndingByMode(mode);
			ln2.MemPts.clear();
			ln2.MemPts = segPnts2;
			flg2 = true;
		}
	}

//	if (DistanceBetweenPoints2D(ln2.BeginPt, ln2.EndPt) < minLength)
//		flg2 = false;

	tmpPts.clear();
	for (int i = 0; i<Pts.size(); i++) {
		bool flg = false;
		for (int j = 0; j<segPnts1.size(); j++) {
			if (Pts[i] == segPnts1[j]) {
				flg = true;
				break;
			}
		}
		for (int j = 0; j<segPnts2.size(); j++) {
			if (Pts[i] == segPnts2[j]) {
				flg = true;
				break;
			}
		}
		if (flg == false)
			tmpPts.push_back(Pts[i]);
	}

	//Third line
	if (tmpPts.size()> minNofPts) {
		//	LineExtractionByRansac(tmpPts,m_ThRansac,ln3,sigma,sptPnts3);
		LineExtractionWithFixedAngle(tmpPts, ang, m_ThRansac, ln3, sptPnts3);
		SegmentPoints(sptPnts3, segPnts3, m_ThDistGrouping);
		if (segPnts3.size()> minNofPts) {
			//		LineExtractionByRansac(segPnts3,m_ThRansac,ln3);
			LineExtractionWithFixedAngle(segPnts3, ang, m_ThRansac, ln3, sptPnts3);
			ln3.SetEndingByMode(mode);
			ln3.MemPts.clear();
			ln3.MemPts = segPnts3;
			flg3 = true;
		}
	}

	double length1, length2, length3;
	length1 = DistanceBetweenPoints2D(ln1.BeginPt, ln1.EndPt);
	length2 = DistanceBetweenPoints2D(ln2.BeginPt, ln2.EndPt);
	length3 = DistanceBetweenPoints2D(ln3.BeginPt, ln3.EndPt);

	//if length of line is shorter than 0.2
	if (length2 < minLength)
		flg2 = false;
	if (length3 < minLength)
		flg3 = false;

	//MDL 
	double closeness1 = 0, closeness2 = 0, closeness3 = 0;
	vector<Line3D> tmpLns;
	tmpLns.push_back(ln1);
	closeness1 = ComputeCloseness(Pts, tmpLns);
	tmpLns.push_back(ln2);
	closeness2 = ComputeCloseness(Pts, tmpLns);
	if (flg2 == false) {
		closeness2 = 1e100;
		closeness3 = 1e100;
	}
	tmpLns.push_back(ln3);
	closeness3 = ComputeCloseness(Pts, tmpLns);
	if (flg3 == false)
		closeness3 = 1e100;

	//MDL
	double wclo = m_wMDL;
	closeness1 = closeness1*m_skip_rate / (2.0*log(2.0));
	closeness2 = closeness2*m_skip_rate / (2.0*log(2.0));
	closeness3 = closeness3*m_skip_rate / (2.0*log(2.0));

	double complexity1 = log10(2.0) / log10(2.0);
	double complexity2 = log10(4.0) / log10(2.0);
	double complexity3 = log10(8.0) / log10(2.0);

	double MDLScore1 = wclo * closeness1 + (1 - wclo) * complexity1;
	double MDLScore2 = wclo * closeness2 + (1 - wclo) * complexity2;
	double MDLScore3 = wclo * closeness3 + (1 - wclo) * complexity3;

	//	printf("%0.3f %0.3f, %0.3f\n",MDLScore1, MDLScore2, MDLScore3);

	if (MDLScore1 < MDLScore2 && MDLScore1 < MDLScore3)
		lnList.push_back(ln1);
	else if (MDLScore2 < MDLScore1 && MDLScore2 < MDLScore3) {
		lnList.push_back(ln1);
		lnList.push_back(ln2);
	}
	else {
		lnList.push_back(ln1);
		lnList.push_back(ln2);
		lnList.push_back(ln3);
	}

	////
	//	for(int i=0;i<lnList.size();i++){
	//		for(int j=0;j<lnList[i].MemPts.size();j++)
	//			lnList[i].MemPts[j]->ClassCode = i+5;
	//	}

	//	for(int i=0;i<lnList.size();i++)
	//		printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",lnList[i].BeginPt.x,lnList[i].BeginPt.y,lnList[i].BeginPt.z,lnList[i].EndPt.x,lnList[i].EndPt.y,lnList[i].EndPt.z);

}

/**
 * @fn	double RailwayExtraction::ComputeCloseness(vector<PointXYZDouble*> Pnts, vector<Line3D> lns)
 *
 * @brief	Calculates the closeness of MDL
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	Pnts	If non-null, the input points.
 * @param 		  	lns 	The hypothesis lines.
 *
 * @return	The calculated closeness.
 */

double RailwayExtraction::ComputeCloseness(vector<PointXYZDouble*> Pnts, vector<Line3D> lns)
{
	double Closeness = 0;
	for (int i = 0; i<Pnts.size(); i++) {
		double minDist = 1e100;
		for (int j = 0; j<lns.size(); j++) {
			double fdist = DistanceBetweenPointAndLine2D(Pnts[i], lns[j]);
			if (fdist < minDist)
				minDist = fdist;
		}
		//	Closeness = Closeness + minDist*minDist;
		Closeness = Closeness + fabs(minDist);
	}
	return Closeness;
}

/**
 * @fn	void RailwayExtraction::DefineTemplates()
 *
 * @brief	Define templates for turnout recognition
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 */

void RailwayExtraction::DefineTemplates()
{
	m_Tmplts.clear();
	m_TmpltBegin.clear();
	m_TmpltEnd.clear();

	BinaryString tmplt;
	tmplt.Type = 0;				//Split to right(start, open)
	tmplt.SetBinaryString(0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 1;				//Split to left (start, open)
	tmplt.SetBinaryString(0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 2;				//Merge from right (start)
	tmplt.SetBinaryString(0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0);
	//	tmplt.SetBinaryString(0,1,0,1,0,1,1,1,0,1,1,0,0,1,0,0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 3;				//Merge from left (start)
	//	tmplt.SetBinaryString(1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,0);
	tmplt.SetBinaryString(0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 4;				//Split to right (start, close)
	tmplt.SetBinaryString(0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 5;				//Split to left (start, close)
	tmplt.SetBinaryString(0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 6;				//Merge from right (end, open)
	tmplt.SetBinaryString(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 7;				//Merge from left (end, open)
	tmplt.SetBinaryString(1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 8;				//Split to right (end)
	tmplt.SetBinaryString(0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0);
	//	tmplt.SetBinaryString(0,1,0,0,0,1,0,0,0,1,0,1,0,1,0,0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 9;				//Split to left (end)
	//	tmplt.SetBinaryString(0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0);
	tmplt.SetBinaryString(0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 10;			//Merge from right (end, close)
	tmplt.SetBinaryString(0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 11;			//Merge from left (end, close)
	tmplt.SetBinaryString(1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 12;			//Split to left (start, both)
	tmplt.SetBinaryString(0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 13;			//Split to right (start, both)
	tmplt.SetBinaryString(0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 14;			//Merge from right (end, both)
	tmplt.SetBinaryString(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 15;			//Merge from left (end, both)
	tmplt.SetBinaryString(1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 16;			//Merge from left (start)
	tmplt.SetBinaryString(0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 17;				//Merge from right (start)
	tmplt.SetBinaryString(0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltBegin.push_back(tmplt);

	tmplt.Type = 18;				//Split to right (end)
	tmplt.SetBinaryString(0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

	tmplt.Type = 19;				//Split to left (end)
	tmplt.SetBinaryString(0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0);
	m_Tmplts.push_back(tmplt);
	m_TmpltEnd.push_back(tmplt);

}

/**
 * @fn	void RailwayExtraction::AdjustTwoLines(Line3D &ln1, Line3D &ln2, int m1, int m2, int m, double _dist)
 *
 * @brief	Adjust parallel rails with fixed track guage
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	ln1  	The first line.
 * @param [in,out]	ln2  	The second line.
 * @param 		  	m1   	The mode of first line.
 * @param 		  	m2   	The mode of second iine.
 * @param 		  	m	 	The reference mode.
 * @param 		  	_dist	The track guage.
 */

void RailwayExtraction::AdjustTwoLines(Line3D &ln1, Line3D &ln2, int m1, int m2, int m, double _dist)
{
	double ang, ang1, ang2;
	ang1 = ln1.Ang;
	ang2 = ln2.Ang;

	//calculate average angle 
	if (m1 == m2)
		ang = (ang1 + ang2) / 2;
	else {
		if (m1 == 1 || m1 == 2) {
			if (m2 == 1 || m2 == 2)
				ang = (ang1 + ang2) / 2;
			else
				ang = (ang1 + (ang2 - 180)) / 2;
		}
		else {
			if (m2 == 1 || m2 == 2) {
				ang = ((ang1 - 180) + ang2) / 2;
			}
			else
				ang = (ang1 + ang2) / 2;
		}
	}

	if (ang < 0)
		ang = ang + 180;

	ln1.GetCenterPt();
	ln2.GetCenterPt();

	POINT3D pt1 = ln1.CenterPt;
	POINT3D pt2 = ln2.CenterPt;

	ln1.Ang = ang;
	ln2.Ang = ang;

	ln1.Get2DLineParameterWithAngleAndPt(pt1.x, pt1.y);
	ln2.Get2DLineParameterWithAngleAndPt(pt2.x, pt2.y);

	pt2 = ComputeFootOfPerpendicular(pt1, ln2);

	Line3D tmpln;
	tmpln.BeginPt = pt1;
	tmpln.EndPt = pt2;
	tmpln.GetLineParameter();
	double fdist = DistanceBetweenPoints2D(tmpln.BeginPt, tmpln.EndPt);
	tmpln = tmpln.ExtendEndingPts(_dist / fdist);
	ln1.Get2DLineParameterWithAngleAndPt(tmpln.BeginPt.x, tmpln.BeginPt.y);
	ln2.Get2DLineParameterWithAngleAndPt(tmpln.EndPt.x, tmpln.EndPt.y);
	ln1.CenterPt = tmpln.BeginPt;
	ln2.CenterPt = tmpln.EndPt;
	ln1.SetEndingPntsByCenterAndLength(m_gLength / 2);
	ln2.SetEndingPntsByCenterAndLength(m_gLength / 2);

	ln1.SetEndingByMode(m);
	ln2.SetEndingByMode(m);

}

/**
 * @fn	void RailwayExtraction::AdjustTwoLinesWithObservation(POINT3D ref1, POINT3D ref2, Line3D &ln1, Line3D &ln2, vector<PointXYZDouble*> sptPts1, vector<PointXYZDouble*> sptPts2, int m1, int m2, int m, double _dist)
 *
 * @brief	Adjust two lines with observation
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	ref1   	The first reference.
 * @param 		  	ref2   	The second reference.
 * @param [in,out]	ln1	   	The first line.
 * @param [in,out]	ln2	   	The second line.
 * @param [in,out]	sptPts1	If non-null, the supporting points for first line.
 * @param [in,out]	sptPts2	If non-null, the supporting points for second line.
 * @param 		  	m1	   	The mode for first line.
 * @param 		  	m2	   	The mode for second line.
 * @param 		  	m	   	The reference mode.
 * @param 		  	_dist  	The track guage.
 */

void RailwayExtraction::AdjustTwoLinesWithObservation(POINT3D ref1, POINT3D ref2, Line3D &ln1, Line3D &ln2, vector<PointXYZDouble*> sptPts1, vector<PointXYZDouble*> sptPts2, int m1, int m2, int m, double _dist)
{
	double ang, ang1, ang2;
	ang1 = ln1.Ang;
	ang2 = ln2.Ang;

	//calculate average angle 
	if (m1 == 1 && m2 == 4) {
		double diff = fabs(ang1 - ang2);
		if (fabs(180 - diff) < diff)
			ang = (ang1 + ang2 + 180) / 2;
		else
			ang = (ang1 + ang2) / 2;
	}
	else if (m1 == 4 && m2 == 1) {
		double diff = fabs(ang1 - ang2);
		if (fabs(180 - diff) < diff)
			ang = (ang1 + ang2 + 180) / 2;
		else
			ang = (ang1 + ang2) / 2;
	}
	else if (m1 == 2 && m2 == 3) {
		double diff = fabs(ang1 - ang2);
		if (fabs(180 - diff) < diff)
			ang = (ang1 + ang2 + 180) / 2;
		else
			ang = (ang1 + ang2) / 2;
	}
	else if (m1 == 3 && m2 == 2) {
		double diff = fabs(ang1 - ang2);
		if (fabs(180 - diff) < diff)
			ang = (ang1 + ang2 + 180) / 2;
		else
			ang = (ang1 + ang2) / 2;
	}
	else {
		ang = (ang1 + ang2) / 2;
	}

	if (ang < 0)
		ang = ang + 180;

	if (ang >= 180)
		ang = ang - 180;

	ln1.GetCenterPt();
	ln2.GetCenterPt();

	POINT3D pt1 = ln1.CenterPt;
	POINT3D pt2 = ln2.CenterPt;

	ln1.Ang = ang;
	ln2.Ang = ang;

	ln1.Get2DLineParameterWithAngleAndPt(pt1.x, pt1.y);
	ln2.Get2DLineParameterWithAngleAndPt(pt2.x, pt2.y);

	pt2 = ComputeFootOfPerpendicular(pt1, ln2);
	////////Weighted average
	vector<POINT3D> DiffList;
	double maxDiff = -1e100, minDiff = 1e100;
	for (int i = 0; i<sptPts1.size(); i++) {
		POINT3D tmpDiff(0, 0, 0);
		double fdist = DistanceBetweenPointAndLine2D(sptPts1[i], ln2);
		tmpDiff.x = fdist;
		tmpDiff.y = fabs(_dist - fdist);
		if (maxDiff < tmpDiff.y)
			maxDiff = tmpDiff.y;
		if (minDiff > tmpDiff.y)
			minDiff = tmpDiff.y;
		DiffList.push_back(tmpDiff);
	}
	for (int i = 0; i<sptPts2.size(); i++) {
		POINT3D tmpDiff(0, 0, 0);
		double fdist = DistanceBetweenPointAndLine2D(sptPts2[i], ln1);
		tmpDiff.x = fdist;
		tmpDiff.y = fabs(_dist - fdist);
		if (maxDiff < tmpDiff.y)
			maxDiff = tmpDiff.y;
		if (minDiff > tmpDiff.y)
			minDiff = tmpDiff.y;
		DiffList.push_back(tmpDiff);
	}
	///nomalization
	for (int i = 0; i<DiffList.size(); i++)
		DiffList[i].z = (DiffList[i].y - minDiff) / (maxDiff - minDiff);

	double weightedDist = 0;
	double weight = 0;
	for (int i = 0; i<DiffList.size(); i++) {
		if (DiffList[i].z == 0) {
			weightedDist = weightedDist + DiffList[i].x;
			weight = weight + 1;
		}
		else {
			weightedDist = weightedDist + DiffList[i].x / DiffList[i].z/*/DiffList[i].z*/;
			weight = weight + 1 / DiffList[i].z/*/DiffList[i].z*/;
		}
	}
	weightedDist = weightedDist / weight;
	//	printf("%0.3f\n",weightedDist);
	////////
	Line3D tmpln;
	tmpln.BeginPt = pt1;
	tmpln.EndPt = pt2;
	tmpln.GetLineParameter();

	tmpln = tmpln.ExtendEndingPts(_dist / weightedDist);
	ln1.Get2DLineParameterWithAngleAndPt(tmpln.BeginPt.x, tmpln.BeginPt.y);
	ln2.Get2DLineParameterWithAngleAndPt(tmpln.EndPt.x, tmpln.EndPt.y);
	ln1.CenterPt = tmpln.BeginPt;
	ln2.CenterPt = tmpln.EndPt;
	ln1.SetEndingPntsByCenterAndLength(m_gLength / 2);
	ln2.SetEndingPntsByCenterAndLength(m_gLength / 2);

	ln1.SetEndByReferencePt(ref1);
	ln2.SetEndByReferencePt(ref2);
	//	ln1.SetEndingByMode(m);
	//	ln2.SetEndingByMode(m);
}

/**
 * @fn	void RailwayExtraction::ValidateLoopCross(vector<LoopCross> &loopcross)
 *
 * @brief	Validates the loop cross detected 
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	loopcross	The list of loopcrosses.
 */

void RailwayExtraction::ValidateLoopCross(vector<LoopCross> &loopcross)
{
	if (loopcross.size() <= 0)
		return;

	vector<vector<int>> cnts;

	for (int i = 0; i<loopcross.size(); i++) {
		int cnt = 0;
		vector<int> ids;
		ids.push_back(i);

		for (int j = 0; j<loopcross.size(); j++) {
			if (i == j)
				continue;
			double fdist = fabs(loopcross[j].distFromSp - loopcross[i].distFromSp);
			int fdist1 = (int)fdist * 100;
			fdist1 = fdist1 % 2500;
			double fdist2 = (double)fdist1 / 100;

			if (fdist2 < 2 || fdist2 > 23) {
				cnt++;
				ids.push_back(j);
			}
		}
		cnts.push_back(ids);
	}

	int max = 0;
	vector<int> lpIdx;
	for (int i = 0; i<cnts.size(); i++) {
		if (cnts[i].size() > max) {
			lpIdx = cnts[i];
			max = cnts[i].size();
		}
	}

	vector<LoopCross> ReLCs;
	for (int i = 0; i<lpIdx.size(); i++)
		ReLCs.push_back(loopcross[lpIdx[i]]);

	loopcross.clear();
	loopcross = ReLCs;

}

/**
 * @fn	void RailwayExtraction::DetectLoopCross(TrackSegment Seg, vector<PointXYZDouble*> Pts, double HgtOfGround, double HgtOfTrack, Line3D refLn, vector<LoopCross> &LC)
 *
 * @brief	Detect loop cross
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	Seg		   	The track segments.
 * @param [in,out]	Pts		   	If non-null, the points.
 * @param 		  	HgtOfGround	The height of ground.
 * @param 		  	HgtOfTrack 	The height of track.
 * @param 		  	refLn	   	The reference line.
 * @param [in,out]	LC		   	The loop cross.
 */

void RailwayExtraction::DetectLoopCross(TrackSegment Seg, vector<PointXYZDouble*> Pts, double HgtOfGround, double HgtOfTrack, Line3D refLn, vector<LoopCross> &LC)
{
	if (Seg.m_Track1.bTurnOut == true || Seg.m_Track2.bTurnOut == true)
		return;

	Line3D tmpLn1 = Seg.m_Track1.MemLn;
	Line3D tmpLn2 = Seg.m_Track2.MemLn;

	vector<PointXYZDouble*> mlPnts, mrPnts, mmPnts;
	Seg.GetNeighborVtxBND(m_gLength + 0.1, 0.50, m_gWidth);
	GetPntsInsidePolygon(Seg.mlVtxBND, Pts, mlPnts);
	GetPntsInsidePolygon(Seg.mrVtxBND, Pts, mrPnts);
	//	GetPntsInsidePolygon(Seg.mmVtxBND,Pts,mmPnts);

	vector<PointXYZDouble*> tmpLoopGrdPts;
	for (int i = 0; i<mlPnts.size(); i++)
		tmpLoopGrdPts.push_back(mlPnts[i]);
	//	for(int i=0;i<mmPnts.size();i++)
	//		tmpLoopGrdPts.push_back(mmPnts[i]);
	for (int i = 0; i<mrPnts.size(); i++)
		tmpLoopGrdPts.push_back(mrPnts[i]);

	double HgtBuffer = 0.1;
	vector<PointXYZDouble*> LoopGrdPts;

	for (int i = 0; i<tmpLoopGrdPts.size(); i++) {
		if (tmpLoopGrdPts[i]->z > HgtOfGround - HgtBuffer)
			continue;
		if (tmpLoopGrdPts[i]->z < HgtOfGround - 2 * HgtBuffer)
			continue;

		double fdist1 = DistanceBetweenPointAndLine2D(tmpLoopGrdPts[i], tmpLn1);
		double fdist2 = DistanceBetweenPointAndLine2D(tmpLoopGrdPts[i], tmpLn2);
		if (fdist1 < 0.3)
			continue;
		if (fdist2 < 0.3)
			continue;
		LoopGrdPts.push_back(tmpLoopGrdPts[i]);
	}

	double ang = refLn.Ang + 90;
	vector<PointXYZDouble*> sptPts;
	Line3D loopLn;
	bool flg = LineExtractionWithFixedAngleForLoopCross(LoopGrdPts, ang, 0.07, loopLn, sptPts);

	double tmpZ1 = loopLn.BeginPt.z;
	double tmpZ2 = loopLn.EndPt.z;
	loopLn.BeginPt = ComputeIntersectionPoint3D(loopLn.A2D, loopLn.B2D, loopLn.C2D, tmpLn1.A2D, tmpLn1.B2D, tmpLn1.C2D);
	loopLn.EndPt = ComputeIntersectionPoint3D(loopLn.A2D, loopLn.B2D, loopLn.C2D, tmpLn2.A2D, tmpLn2.B2D, tmpLn2.C2D);
	loopLn.BeginPt.z = tmpZ1;
	loopLn.EndPt.z = tmpZ2;
	loopLn.GetLineParameter();
	loopLn.GetCenterPt();

	if (flg == true) {
		LoopCross tmpLC;
		tmpLC.lc = loopLn;
		tmpLC.distFromSp = Seg.distFromSp - DistanceBetweenPoints2D(loopLn.CenterPt, refLn.EndPt);

		if (LC.size() == 0)
			LC.push_back(tmpLC);
		else {
			Line3D lastLn = LC[LC.size() - 1].lc;
			double fdist1 = DistanceBetweenPointAndLine2D(loopLn.CenterPt, lastLn);
			double fdist2 = DistanceBetweenPointAndLine2D(lastLn.CenterPt, loopLn);
			if ((fdist1 + fdist2) / 2 > 1)
				LC.push_back(tmpLC);
		}
	}

	//visualization
	//	for(int i=0;i<LoopGrdPts.size();i++)
	//		LoopGrdPts[i]->ClassCode = 3;
	//	for(int i=0;i<LoopGrdPts.size();i++)
	//		LoopGrdPts[i]->ClassCode = 3;

}

/**
 * @fn	void RailwayExtraction::DetectSwitchBox(vector<ToePt> toes, vector<SwitchBox> &SBs)
 *
 * @brief	Detect switch box
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	toes	The toes.
 * @param [in,out]	SBs 	The switch boxes.
 */

void RailwayExtraction::DetectSwitchBox(vector<ToePt> toes, vector<SwitchBox> &SBs)
{
	double _gSize = m_GridSize;
	vector<PointXYZDouble*> rgnPts;
	double orthoDist = 2;
	double ThOrthoDist = 0.5;
	double ThAlongDist = 1.0;

	//make candidates of switch box
	vector<SwitchBox> candidateSBs;
	for (int i = 0; i<toes.size(); i++) {
		if (toes[i].used == true)
			continue;
		SwitchBox rgn1, rgn2;
		Line3D oLn;
		oLn.BeginPt = toes[i].RightPt;
		oLn.EndPt = toes[i].LeftPt;
		oLn.GetLineParameter();
		oLn.GetAngle();
		oLn.GetCenterPt();
		oLn = oLn.ExtendEndingPts(2.53);
		oLn.GetLineParameter();
		oLn.GetAngle();
		oLn.GetCenterPt();

		Line3D aLn1, aLn2;
		aLn1.CenterPt = oLn.BeginPt;
		aLn2.CenterPt = oLn.EndPt;
		aLn1.Ang = oLn.Ang;
		aLn2.Ang = oLn.Ang;

		aLn1.SetEndingPntsByCenterAndLength(ThOrthoDist);
		aLn2.SetEndingPntsByCenterAndLength(ThOrthoDist);
		aLn1.GetLineParameter();
		aLn2.GetLineParameter();

		rgn1.rgn = aLn1.GetBufferVtxOfLine(ThAlongDist);
		rgn2.rgn = aLn2.GetBufferVtxOfLine(ThAlongDist);
		rgn1.CenterPt = aLn1.CenterPt;
		rgn2.CenterPt = aLn2.CenterPt;
		rgn1.ang = aLn1.Ang;
		rgn2.ang = aLn2.Ang;
		rgn1.referHgt = (oLn.BeginPt.z + oLn.EndPt.z) / 2;
		rgn2.referHgt = (oLn.BeginPt.z + oLn.EndPt.z) / 2;

		candidateSBs.push_back(rgn1);
		candidateSBs.push_back(rgn2);
	}

	//test
	//	SBs = candidateSBs;

	//recognize switch Box 
	for (int i = 0; i<candidateSBs.size(); i++) {
		//Assign member points
		vector<int> neighborIdx = GetNeighborIdx(candidateSBs[i].CenterPt.x, candidateSBs[i].CenterPt.y, m_NumX, m_NumY, _gSize, 3);
		rgnPts.clear();
		for (int j = 0; j<neighborIdx.size(); j++) {
			if (neighborIdx[j]<0 || neighborIdx[j] >= m_NumGrid)
				continue;
			for (int k = 0; k<m_pGrid[neighborIdx[j]].MemPnts.size(); k++)
				rgnPts.push_back(m_pGrid[neighborIdx[j]].MemPnts[k]);
		}
		vector<PointXYZDouble*> tmpRgnPts;
		GetPntsInsidePolygon(candidateSBs[i].rgn, rgnPts, tmpRgnPts);
		double ThUpper = candidateSBs[i].referHgt + 0.10;
		double ThLower = candidateSBs[i].referHgt - 0.00;

		vector<PointXYZDouble*> tmpSBPts, SBPts, supportPts;
		for (int j = 0; j<tmpRgnPts.size(); j++) {
			if (tmpRgnPts[j]->z < ThUpper && tmpRgnPts[j]->z > ThLower)
				tmpSBPts.push_back(tmpRgnPts[j]);
		}

		PlnPara para;
		ExtractPlaneByRANSAC(tmpSBPts, para, 0.008, supportPts);
		SegmentPoints(supportPts, SBPts, 0.1);	//0.05
		ExtractPlaneByRANSAC(SBPts, para, 0.008, supportPts);

		//constraints
		if (fabs(para.c) < 0.999)
			continue;
		if (SBPts.size() < 1000 / m_skip_rate)
			continue;

		for (int j = 0; j<SBPts.size(); j++)
			SBPts[j]->ClassCode = 5;

		//final switch box
		POINT3D center(0, 0, 0);
		for (int j = 0; j<SBPts.size(); j++) {
			center.x = center.x + SBPts[j]->x;
			center.y = center.y + SBPts[j]->y;
			center.z = center.z + SBPts[j]->z;
		}
		center = center / SBPts.size();
		SwitchBox sb;
		sb.CenterPt = center;
		sb.ang = candidateSBs[i].ang;
		sb.GenerateRgnsUsingCenterPt(0.7);
		for (int j = 0; j<sb.rgn.size(); j++) {
			sb.rgn[j].z = center.z;
		}
		SBs.push_back(sb);
		currentLayer->m_SwitchBoxes.push_back(sb);
	}
}

/**
 * @fn	bool RailwayExtraction::DetectToePnts(int type, vector<TrackSegment> trkSeg, ToePt &toe)
 *
 * @brief	Detect toe pnts
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	type  	The turnout type.
 * @param 		  	trkSeg	The list of track segments.
 * @param [in,out]	toe   	The toe.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::DetectToePnts(int type, vector<TrackSegment> trkSeg, ToePt &toe)
{
	vector<TrackSegment> trkseg = trkSeg;
	bool flg = false;

	if (type == 0 || type == 1 || type == 4 || type == 5 || type == 6 || type == 7 || type == 10 ||
		type == 11 || type == 12 || type == 13 || type == 14 || type == 15)
		flg = true;
	else
		flg = false;

	if (flg == false)
		return false;

	//////
	int end = trkSeg.size() - 1;
	toe.TrkType = type;

	vector<PointXYZDouble*> tmpPnts;
	vector<PointXYZDouble*> segmentPts;
	vector<PointXYZDouble*> sptPnts;
	Line3D ln;
	Line3D ln_track1, ln_track2;
	double sigma;
	double maxX, minX, maxY, minY;
	double ThDist = 0.03;
	if (toe.TrkType == 0 || toe.TrkType == 1 || toe.TrkType == 12 || toe.TrkType == 13) {
		for (int i = 0; i<trkSeg[0].m_Track2.srMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[0].m_Track2.srMemPnts[i]);
		for (int i = 0; i<trkSeg[1].m_Track2.srMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[1].m_Track2.srMemPnts[i]);
		SegmentPoints(tmpPnts, segmentPts, ThDist);

		LineExtractionByRansac(segmentPts, m_ThRansac, ln, sigma, sptPnts);
		GetMinMaxXY(sptPnts, minX, maxX, minY, maxY);
		ln.SetEndingPnts(minX, maxX, minY, maxY);
		POINT3D Refpt = trkSeg[0].m_Track2.MemLn.BeginPt;
		double fdist1 = DistanceBetweenPoints2D(Refpt, ln.BeginPt);
		double fdist2 = DistanceBetweenPoints2D(Refpt, ln.EndPt);
		POINT3D tmpPt;
		if (fdist1 < fdist2)
			tmpPt = ln.BeginPt;
		else
			tmpPt = ln.EndPt;

		ln_track1 = trkSeg[0].m_Track1.MemLn;
		ln_track2 = trkSeg[0].m_Track2.MemLn;
		ln_track1.GetAngle();

		//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln_track1.BeginPt.x,ln_track1.BeginPt.y,ln_track1.BeginPt.z,ln_track1.EndPt.x,ln_track1.EndPt.y,ln_track1.EndPt.z);
		//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln_track2.BeginPt.x,ln_track2.BeginPt.y,ln_track2.BeginPt.z,ln_track2.EndPt.x,ln_track2.EndPt.y,ln_track2.EndPt.z);

		toe.RightPt = ComputeFootOfPerpendicular(tmpPt, ln_track1);
		toe.LeftPt = ComputeFootOfPerpendicular(tmpPt, ln_track2);
		toe.CenterPt = (toe.RightPt + toe.LeftPt) / 2;
		toe.TrkMode = GetDirectionMode(ln_track1.EndPt.x, ln_track1.EndPt.y, ln_track1.BeginPt.x, ln_track1.BeginPt.y);
		toe.TrkAng = ln_track1.Ang;

		//	printf("%d %0.3f\n",toe.TrkMode, toe.TrkAng);
		//	printf("%0.3f %0.3f %0.3f\n",toe.RightPt.x, toe.RightPt.y, toe.RightPt.z);
		//	printf("%0.3f %0.3f %0.3f\n",toe.LeftPt.x, toe.LeftPt.y, toe.LeftPt.z);

	}
	else if (toe.TrkType == 4 || toe.TrkType == 5) {
		for (int i = 0; i<trkSeg[0].m_Track1.slMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[0].m_Track1.slMemPnts[i]);
		for (int i = 0; i<trkSeg[1].m_Track1.slMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[1].m_Track1.slMemPnts[i]);
		SegmentPoints(tmpPnts, segmentPts, ThDist);

		LineExtractionByRansac(segmentPts, m_ThRansac, ln, sigma, sptPnts);
		GetMinMaxXY(sptPnts, minX, maxX, minY, maxY);
		ln.SetEndingPnts(minX, maxX, minY, maxY);
		POINT3D Refpt = trkSeg[0].m_Track1.MemLn.BeginPt;
		double fdist1 = DistanceBetweenPoints2D(Refpt, ln.BeginPt);
		double fdist2 = DistanceBetweenPoints2D(Refpt, ln.EndPt);
		POINT3D tmpPt;
		if (fdist1 < fdist2)
			tmpPt = ln.BeginPt;
		else
			tmpPt = ln.EndPt;

		ln_track1 = trkSeg[0].m_Track1.MemLn;
		ln_track2 = trkSeg[0].m_Track2.MemLn;
		ln_track1.GetAngle();

		toe.RightPt = ComputeFootOfPerpendicular(tmpPt, ln_track1);
		toe.LeftPt = ComputeFootOfPerpendicular(tmpPt, ln_track2);
		toe.CenterPt = (toe.RightPt + toe.LeftPt) / 2;
		toe.TrkMode = GetDirectionMode(ln_track1.EndPt.x, ln_track1.EndPt.y, ln_track1.BeginPt.x, ln_track1.BeginPt.y);
		toe.TrkAng = ln_track1.Ang;
	}
	else if (toe.TrkType == 6 || toe.TrkType == 7 || toe.TrkType == 14 || toe.TrkType == 15) {
		for (int i = 0; i<trkSeg[end - 1].m_Track2.srMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[end - 1].m_Track2.srMemPnts[i]);
		for (int i = 0; i<trkSeg[end].m_Track2.srMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[end].m_Track2.srMemPnts[i]);
		SegmentPoints(tmpPnts, segmentPts, ThDist);

		LineExtractionByRansac(segmentPts, m_ThRansac, ln, sigma, sptPnts);
		GetMinMaxXY(sptPnts, minX, maxX, minY, maxY);
		ln.SetEndingPnts(minX, maxX, minY, maxY);
		POINT3D Refpt = trkSeg[end].m_Track2.MemLn.BeginPt;
		double fdist1 = DistanceBetweenPoints2D(Refpt, ln.BeginPt);
		double fdist2 = DistanceBetweenPoints2D(Refpt, ln.EndPt);
		POINT3D tmpPt;
		if (fdist1 > fdist2)
			tmpPt = ln.BeginPt;
		else
			tmpPt = ln.EndPt;

		ln_track1 = trkSeg[end].m_Track1.MemLn;
		ln_track2 = trkSeg[end].m_Track2.MemLn;
		ln_track1.GetAngle();

		toe.RightPt = ComputeFootOfPerpendicular(tmpPt, ln_track1);
		toe.LeftPt = ComputeFootOfPerpendicular(tmpPt, ln_track2);
		toe.CenterPt = (toe.RightPt + toe.LeftPt) / 2;
		toe.TrkMode = GetDirectionMode(ln_track1.EndPt.x, ln_track1.EndPt.y, ln_track1.BeginPt.x, ln_track1.BeginPt.y);
		toe.TrkAng = ln_track1.Ang;

		//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln_track1.BeginPt.x,ln_track1.BeginPt.y,ln_track1.BeginPt.z,ln_track1.EndPt.x,ln_track1.EndPt.y,ln_track1.EndPt.z);
		//	printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n",ln_track2.BeginPt.x,ln_track2.BeginPt.y,ln_track2.BeginPt.z,ln_track2.EndPt.x,ln_track2.EndPt.y,ln_track2.EndPt.z);
	}
	else if (toe.TrkType == 10 || toe.TrkType == 11) {
		for (int i = 0; i<trkSeg[end - 1].m_Track1.slMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[end - 1].m_Track1.slMemPnts[i]);
		for (int i = 0; i<trkSeg[end].m_Track1.slMemPnts.size(); i++)
			tmpPnts.push_back(trkSeg[end].m_Track1.slMemPnts[i]);
		SegmentPoints(tmpPnts, segmentPts, ThDist);

		LineExtractionByRansac(segmentPts, m_ThRansac, ln, sigma, sptPnts);
		GetMinMaxXY(sptPnts, minX, maxX, minY, maxY);
		ln.SetEndingPnts(minX, maxX, minY, maxY);
		POINT3D Refpt = trkSeg[end].m_Track1.MemLn.BeginPt;
		double fdist1 = DistanceBetweenPoints2D(Refpt, ln.BeginPt);
		double fdist2 = DistanceBetweenPoints2D(Refpt, ln.EndPt);
		POINT3D tmpPt;
		if (fdist1 > fdist2)
			tmpPt = ln.BeginPt;
		else
			tmpPt = ln.EndPt;

		ln_track1 = trkSeg[end].m_Track1.MemLn;
		ln_track2 = trkSeg[end].m_Track2.MemLn;
		ln_track1.GetAngle();

		toe.RightPt = ComputeFootOfPerpendicular(tmpPt, ln_track1);
		toe.LeftPt = ComputeFootOfPerpendicular(tmpPt, ln_track2);
		toe.CenterPt = (toe.RightPt + toe.LeftPt) / 2;
		toe.TrkMode = GetDirectionMode(ln_track1.EndPt.x, ln_track1.EndPt.y, ln_track1.BeginPt.x, ln_track1.BeginPt.y);
		toe.TrkAng = ln_track1.Ang;
	}
	else
		return false;

	return flg;
}

/**
 * @fn	void RailwayExtraction::RecognitionTurnoutType(vector<BinaryString> tmplates, BinaryString preBinary, BinaryString &trkbinary)
 *
 * @brief	Recognition turnout type
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	tmplates 	The tmplates.
 * @param 		  	preBinary	The binary string for previous track stack.
 * @param [in,out]	trkbinary	The binary string for current track stack.
 */

void RailwayExtraction::RecognitionTurnoutType(vector<BinaryString> tmplates, BinaryString preBinary, BinaryString &trkbinary)
{
	for (int i = 0; i<trkbinary.bStrings.size(); i++)
		printf("%d ", trkbinary.bStrings[i]);
	printf("\n");


	trkbinary.Type = -1;
	vector<double> UnaryScore;
	vector<double> ContextScore;
	vector<double> Score;
	double weight = 1;	//0.9

	for (int i = 0; i<tmplates.size(); i++) {
		double tmpUnaryScore = 0;
		double tmpContextScore = 0;

		//unary term 
		for (int j = 0; j<tmplates[i].bStrings.size(); j++) {
			if (tmplates[i].bStrings[j] == trkbinary.bStrings[j])
				tmpUnaryScore++;
		}
		UnaryScore.push_back(tmpUnaryScore);

		//contextual term
		if (tmplates[i].LabelPosTurnout != preBinary.LabelPosTurnout)
			tmpContextScore = tmpContextScore + 1;
		if (preBinary.LabelPosTurnout == 0) {
			if (tmplates[i].LabelDirection == preBinary.LabelDirection)
				tmpContextScore = tmpContextScore + 1;
			if (tmplates[i].LabelTurnoutType == preBinary.LabelTurnoutType)
				tmpContextScore = tmpContextScore + 1;
		}
		ContextScore.push_back(tmpContextScore);
	}

	//compute total score 
	for (int i = 0; i<UnaryScore.size(); i++) {
		double tmpScore = 0;
		tmpScore = weight*UnaryScore[i] / 16 + (1 - weight)*ContextScore[i];
		Score.push_back(tmpScore);
	}

	//	for(int i=0;i<Score.size();i++)
	//		printf("Unary: %0.2f, Contextual: %0.2f, Total Score: %0.2f\n",UnaryScore[i],ContextScore[i],Score[i]);
	//	printf("\n");

	double maxScore = 0;
	for (int i = 0; i<Score.size(); i++) {
		if (maxScore < Score[i])
			maxScore = Score[i];
	}

	int cnt = 0;
	for (int i = 0; i<Score.size(); i++) {
		if (maxScore == Score[i])
			cnt++;
	}

	if (cnt > 1) {
		trkbinary.Type = 999;
		trkbinary.Score = maxScore;
		PrintTurnoutType(999);
	}
	else {
		for (int i = 0; i<Score.size(); i++) {
			if (maxScore == Score[i]) {
				PrintTurnoutType(tmplates[i].Type);
				if (trkbinary.Type < 0) {
					trkbinary.Type = tmplates[i].Type;
					trkbinary.Score = Score[i];
				}
			}
		}
	}


	trkbinary.SetLables(trkbinary.Type);

	printf("Template ID: %d, Score: %0.2f\n", trkbinary.Type, trkbinary.Score);
	printf("\n\n");
}

/**
 * @fn	void RailwayExtraction::TemplateMatching(vector<BinaryString> tmplates, BinaryString &trkbinary)
 *
 * @brief	Template matching
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	tmplates 	The binary string for the tmplates.
 * @param [in,out]	trkbinary	The binary string for track stack.
 */

void RailwayExtraction::TemplateMatching(vector<BinaryString> tmplates, BinaryString &trkbinary)
{
	for (int i = 0; i<trkbinary.bStrings.size(); i++)
		printf("%d ", trkbinary.bStrings[i]);
	printf("\n");

	trkbinary.Type = -1;
	vector<int> score;
	for (int i = 0; i<tmplates.size(); i++) {
		int tmpScore = 0;
		for (int j = 0; j<tmplates[i].bStrings.size(); j++) {
			if (tmplates[i].bStrings[j] == trkbinary.bStrings[j])
				tmpScore++;
		}
		score.push_back(tmpScore);
	}

	int maxScore = 0;
	for (int i = 0; i<score.size(); i++) {
		if (maxScore < score[i])
			maxScore = score[i];
	}

	for (int i = 0; i<score.size(); i++)
		printf("%d ", score[i]);
	printf("\n");

	for (int i = 0; i<score.size(); i++) {
		if (maxScore == score[i]) {
			PrintTurnoutType(tmplates[i].Type);
			if (trkbinary.Type < 0) {
				trkbinary.Type = tmplates[i].Type;
				trkbinary.Score = score[i];
			}
		}
	}
	printf("Template ID: %d, Score: %0.0f\n", trkbinary.Type, trkbinary.Score);
	printf("\n\n");

}

/**
 * @fn	bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line, double &sigma, vector<PointXYZDouble*> &sptPnts)
 *
 * @brief	Line extraction by ransac
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pnts   	If non-null, the input points.
 * @param 		  	ThDist 	The threshold for distance.
 * @param [in,out]	_line  	The extracted line.
 * @param [in,out]	sigma  	The sigma.
 * @param [in,out]	sptPnts	[in,out] If non-null, the supporting points.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line, double &sigma, vector<PointXYZDouble*> &sptPnts)
{
	sptPnts.clear();
	vector<PointXYZDouble*> tmpPnts;
	double minX = 1e100, minY = 1e100, maxX = -1e100, maxY = -1e100;
	for (int i = 0; i<pnts.size(); i++) {
		if (minX>pnts[i]->x) minX = pnts[i]->x;
		if (maxX<pnts[i]->x) maxX = pnts[i]->x;
		if (minY>pnts[i]->y) minY = pnts[i]->y;
		if (maxY<pnts[i]->y) maxY = pnts[i]->y;
		tmpPnts.push_back(pnts[i]);
	}

	if (tmpPnts.size() < 2)
		return false;

	int MaxIter = 500;
	int MaxCount = 0;

	//	vector<LiDPOINT*> sptPnts;
	vector<PointXYZDouble*> tmpsptPnts;	//support points
	for (int i = 0; i<MaxIter; i++) {
		int spidx = rand() % tmpPnts.size();
		int epidx = rand() % tmpPnts.size();
		if (spidx == epidx)
			continue;

		tmpsptPnts.clear();

		POINT3D sp, ep;
		sp.x = tmpPnts[spidx]->x;
		sp.y = tmpPnts[spidx]->y;
		sp.z = tmpPnts[spidx]->z;
		ep.x = tmpPnts[epidx]->x;
		ep.y = tmpPnts[epidx]->y;
		ep.z = tmpPnts[epidx]->z;

		Line3D tmpline;
		tmpline.BeginPt = sp;
		tmpline.EndPt = ep;
		tmpline.GetLineParameter();

		int count = 0;
		for (int j = 0; j<tmpPnts.size(); j++) {
			POINT3D pt;
			pt.x = tmpPnts[j]->x;
			pt.y = tmpPnts[j]->y;
			pt.z = tmpPnts[j]->z;

			double fdist = DistanceBetweenPointAndLine2D(pt, tmpline);
			//	double fdist = DistanceBetweenPointAndLine3D(pt,tmpline);
			if (fdist < ThDist) {
				count++;
				tmpsptPnts.push_back(tmpPnts[j]);
			}
		}

		if (count > MaxCount) {
			MaxCount = count;
			sptPnts.clear();
			sptPnts = tmpsptPnts;
			_line.BeginPt = sp;
			_line.EndPt = ep;
		}
	}

	_line.GetLineParameter();

	if (sptPnts.size() < 2)
		return false;

	//	LeastSqureForLinearFit(sptPnts,_line);
	//	for(int i=0;i<sptPnts.size();i++)
	//		printf("%0.3f %0.3f %0.3f\n",sptPnts[i]->x, sptPnts[i]->y, sptPnts[i]->z);

	//define ending points
	sort(sptPnts.begin(), sptPnts.end(), HigherZValue);
	int HgtIdx = (int)(sptPnts.size()*0.05);

	_line.SetEndingPnts3D(minX, maxX, minY, maxY);
	_line.BeginPt.z = sptPnts[HgtIdx]->z;
	_line.EndPt.z = sptPnts[HgtIdx]->z;
	_line.GetCenterPt();
	_line.GetAngle();
	_line.GetLineParameter();


	//Compute sigma
	double tmpSig = 0;
	for (int i = 0; i<sptPnts.size(); i++) {
		POINT3D pt;
		pt.x = sptPnts[i]->x;
		pt.y = sptPnts[i]->y;
		pt.z = sptPnts[i]->z;
		double fdist = DistanceBetweenPointAndLine2D(pt, _line);
		tmpSig = tmpSig + fdist*fdist;
	}
	tmpSig = tmpSig / sptPnts.size();
	sigma = sqrt(tmpSig);

	return true;
}

/**
 * @fn	bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line)
 *
 * @brief	Line extraction by ransac
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pnts  	If non-null, the input points.
 * @param 		  	ThDist	The threshold for distance.
 * @param [in,out]	_line 	The extracted line.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line)
{
	vector<PointXYZDouble*> tmpPnts;
	double minX = 1e100, minY = 1e100, maxX = -1e100, maxY = -1e100;
	for (int i = 0; i<pnts.size(); i++) {
		if (minX>pnts[i]->x) minX = pnts[i]->x;
		if (maxX<pnts[i]->x) maxX = pnts[i]->x;
		if (minY>pnts[i]->y) minY = pnts[i]->y;
		if (maxY<pnts[i]->y) maxY = pnts[i]->y;
		tmpPnts.push_back(pnts[i]);
	}

	if (tmpPnts.size() < 2)
		return false;

	int MaxIter = 100;
	int MaxCount = 0;

	vector<PointXYZDouble*> sptPnts;
	vector<PointXYZDouble*> tmpsptPnts;	//support points
	for (int i = 0; i<MaxIter; i++) {
		int spidx = rand() % tmpPnts.size();
		int epidx = rand() % tmpPnts.size();
		if (spidx == epidx)
			continue;

		tmpsptPnts.clear();

		POINT3D sp, ep;
		sp.x = tmpPnts[spidx]->x;
		sp.y = tmpPnts[spidx]->y;
		sp.z = tmpPnts[spidx]->z;
		ep.x = tmpPnts[epidx]->x;
		ep.y = tmpPnts[epidx]->y;
		ep.z = tmpPnts[epidx]->z;

		Line3D tmpline;
		tmpline.BeginPt = sp;
		tmpline.EndPt = ep;
		tmpline.GetLineParameter();

		int count = 0;
		for (int j = 0; j<tmpPnts.size(); j++) {
			POINT3D pt;
			pt.x = tmpPnts[j]->x;
			pt.y = tmpPnts[j]->y;
			pt.z = tmpPnts[j]->z;

			double fdist = DistanceBetweenPointAndLine2D(pt, tmpline);
			//	double fdist = DistanceBetweenPointAndLine3D(pt,tmpline);
			if (fdist < ThDist) {
				count++;
				tmpsptPnts.push_back(tmpPnts[j]);
			}
		}

		if (count > MaxCount) {
			MaxCount = count;
			sptPnts.clear();
			sptPnts = tmpsptPnts;
			_line.BeginPt = sp;
			_line.EndPt = ep;
		}
	}

	_line.GetLineParameter();

	//define ending points
	sort(sptPnts.begin(), sptPnts.end(), HigherZValue);
	int HgtIdx = (int)(sptPnts.size()*0.15);

	_line.SetEndingPnts3D(minX, maxX, minY, maxY);
	_line.BeginPt.z = sptPnts[HgtIdx]->z;
	_line.EndPt.z = sptPnts[HgtIdx]->z;
	_line.GetCenterPt();
	_line.GetAngle();
	_line.GetLineParameter();

	return true;
}

/**
 * @fn	bool RailwayExtraction::LineExtractionWithFixedAngle(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_ln, vector<PointXYZDouble*> &sptPnts)
 *
 * @brief	Line extraction with fixed angle
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pnts   	If non-null, the input points.
 * @param 		  	ang	   	The reference angle.
 * @param 		  	ThDist 	The threshold for distance.
 * @param [in,out]	_ln	   	The extracted line.
 * @param [in,out]	sptPnts	[in,out] If non-null, the supporting points.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::LineExtractionWithFixedAngle(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_ln, vector<PointXYZDouble*> &sptPnts)
{
	int MaxIter = pnts.size();
	int MaxCnt = 0;

	if (pnts.size() < 1)
		return false;

	vector<PointXYZDouble*> tmpsptPnts;

	for (int i = 0; i<MaxIter; i++) {
		int idx = rand() % pnts.size();
		tmpsptPnts.clear();
		Line3D tmpLn;
		tmpLn.Ang = ang;
		tmpLn.Get2DLineParameterWithAngleAndPt(pnts[idx]->x, pnts[idx]->y);

		double cnt = 0;
		for (int j = 0; j<pnts.size(); j++) {
			double fdist = DistanceBetweenPointAndLine2D(pnts[j], tmpLn);
			if (fdist < ThDist) {
				cnt++;
				tmpsptPnts.push_back(pnts[j]);
			}
		}

		if (cnt <= MaxCnt)
			continue;

		MaxCnt = cnt;
		sptPnts.clear();
		sptPnts = tmpsptPnts;
		_ln = tmpLn;
	}

	double maxX, minX, maxY, minY;
	GetMinMaxXY(sptPnts, minX, maxX, minY, maxY);
	_ln.SetEndingPnts(minX, maxX, minY, maxY);

	sort(sptPnts.begin(), sptPnts.end(), HigherZValue);
	int HgtIdx = (int)(sptPnts.size()*0.05);

	_ln.BeginPt.z = sptPnts[HgtIdx]->z;
	_ln.EndPt.z = sptPnts[HgtIdx]->z;
	_ln.GetCenterPt();
	_ln.GetAngle();
	_ln.GetLineParameter();

	return true;

}

/**
 * @fn	bool RailwayExtraction::LineExtractionWithFixedAngleForLoopCross(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_line, vector<PointXYZDouble*> &sptPnts)
 *
 * @brief	Line extraction with fixed angle for loop cross
 *
 * @author	Jaewook Jung
 * @date	10/27/2017
 *
 * @param [in,out]	pnts   	If non-null, the input points.
 * @param 		  	ang	   	The reference angle.
 * @param 		  	ThDist 	The threshold for distance.
 * @param [in,out]	_line  	The output line.
 * @param [in,out]	sptPnts	[in,out] If non-null, the support points.
 *
 * @return	True if it succeeds, false if it fails.
 */

bool RailwayExtraction::LineExtractionWithFixedAngleForLoopCross(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_line, vector<PointXYZDouble*> &sptPnts)
{
	vector<PointXYZDouble*> tmpPnts;
	for (int i = 0; i<pnts.size(); i++)
		tmpPnts.push_back(pnts[i]);

	if (tmpPnts.size() < 1)
		return false;

	int MaxIter = tmpPnts.size();
	int MaxCount = 0;

	vector<PointXYZDouble*> tmpsptPnts;	//support points
	for (int i = 0; i<MaxIter; i++) {
		int idx = rand() % tmpPnts.size();
		tmpsptPnts.clear();
		Line3D tmpLn;
		tmpLn.Ang = ang;
		tmpLn.Get2DLineParameterWithAngleAndPt(tmpPnts[idx]->x, tmpPnts[idx]->y);

		double count = 0;
		double count1 = 0;
		for (int j = 0; j<tmpPnts.size(); j++) {
			POINT3D pt;
			pt.x = tmpPnts[j]->x;
			pt.y = tmpPnts[j]->y;
			pt.z = tmpPnts[j]->z;
			double fdist = DistanceBetweenPointAndLine2D(pt, tmpLn);

			if (fdist < ThDist) {
				count++;
				tmpsptPnts.push_back(tmpPnts[j]);
			}
			if (fdist < ThDist * 2)
				count1++;
		}

		if (count*1.4 < count1)
			continue;

		if (count <= MaxCount)
			continue;

		MaxCount = count;
		sptPnts.clear();
		sptPnts = tmpsptPnts;
		_line = tmpLn;

	}

	double ThnOfPts = 20 / m_skip_rate;
	if (ThnOfPts < 15)
		ThnOfPts = 15;

	if (sptPnts.size()<ThnOfPts)
		return false;

	double minX = 1e100, minY = 1e100, maxX = -1e100, maxY = -1e100;
	for (int i = 0; i<sptPnts.size(); i++) {
		if (minX>sptPnts[i]->x) minX = sptPnts[i]->x;
		if (maxX<sptPnts[i]->x) maxX = sptPnts[i]->x;
		if (minY>sptPnts[i]->y) minY = sptPnts[i]->y;
		if (maxY<sptPnts[i]->y) maxY = sptPnts[i]->y;
	}

	//define ending points
	sort(sptPnts.begin(), sptPnts.end(), HigherZValue);
	int HgtIdx = (int)(sptPnts.size()*0.05);

	_line.SetEndingPnts3D(minX, maxX, minY, maxY);
	_line.BeginPt.z = sptPnts[HgtIdx]->z;
	_line.EndPt.z = sptPnts[HgtIdx]->z;
	_line.GetCenterPt();
	_line.GetAngle();
	_line.GetLineParameter();

	double fdist = DistanceBetweenPoints2D(_line.BeginPt, _line.EndPt);
	if (fdist < 0.5)
		return false;

	return true;
}

/**
 * @fn	void RailwayExtraction::ComputeGMMfeature(vector<PointXYZDouble*> pnts, vector<GMMPara> &paras, int _nClass)
 *
 * @brief	Calculates the gmm features
 *
 * @author	Jaewook Jung
 * @date	10/27/2017
 *
 * @param [in,out]	pnts   	If non-null, the input points.
 * @param [in,out]	paras  	The output gmm parameters.
 * @param 		  	_nClass	The the number of classes.
 */

void RailwayExtraction::ComputeGMMfeature(vector<PointXYZDouble*> pnts, vector<GMMPara> &paras, int _nClass)
{
	int nClass = _nClass;

	//Initialize EM Parameter
	vector<double> mean;
	vector<double> sigma;
	vector<double> alpha;

	double minHeight = 1e100;
	double maxHeight = -1e100;
	for (int i = 0; i<pnts.size(); i++) {
		if (minHeight>pnts[i]->z)
			minHeight = pnts[i]->z;
		if (maxHeight<pnts[i]->z)
			maxHeight = pnts[i]->z;
	}

	//	printf("maxHeight:%0.3f, minHeighgt:%0.3f\n",maxHeight, minHeight);
	double tmpDiffrate = (maxHeight - minHeight) / nClass;

	for (int i = 0; i<nClass; i++) {
		mean.push_back(minHeight + i*tmpDiffrate);
		sigma.push_back(1);
		alpha.push_back(double(1. / nClass));
	}

	int nSample = pnts.size();
	vector<vector<double> > prob_cls;
	for (unsigned int i = 0; i<nSample; ++i) {
		vector<double> tmpClass;
		for (int j = 0; j<nClass; j++)
			tmpClass.push_back(0.0);
		prob_cls.push_back(tmpClass);
	}

	int nIter = 0;
	bool flg = true;
	while (flg == true) {
		///some stop condition(added later) 
		bool flg1 = false;
		for (int i = 0; i<sigma.size(); i++) {
			if (sigma[i] == 0)
				flg1 = true;
		}
		if (fabs(mean[nClass - 1] - mean[nClass - 2])< 0.05)
			flg1 = true;
		if (flg1 == true)
			break;
		////////////

		for (int i = 0; i<nSample; i++) {
			for (int j = 0; j<nClass; j++) {
				double diff = pnts[i]->z - mean[j];
				prob_cls[i][j] = 1 / (sigma[j] * sqrt(2 * PI))*exp(-1 * (pnts[i]->z - mean[j])*(pnts[i]->z - mean[j]) / (2 * sigma[j] * sigma[j])) * alpha[j];
			}
		}
		for (int i = 0; i<nSample; i++) {
			double tmpHop = 0;
			for (int j = 0; j<nClass; j++)
				tmpHop += prob_cls[i][j];
			for (int j = 0; j<nClass; j++)
				prob_cls[i][j] = prob_cls[i][j] / tmpHop;
		}

		//Maximization
		vector<double> nextMean;
		vector<double> nextSigma;
		vector<double> nextAlpha;

		for (int i = 0; i<nClass; i++) {
			nextMean.push_back(0);
			nextSigma.push_back(0);
			nextAlpha.push_back(0);
		}

		for (int i = 0; i<nClass; i++) {
			double ave = 0;
			double weightave = 0;
			for (int j = 0; j<nSample; j++) {
				ave += prob_cls[j][i];
				weightave += prob_cls[j][i] * pnts[j]->z;
			}
			nextAlpha[i] = ave / nSample;
			nextMean[i] = weightave / ave;

			double weightsigma = 0;
			for (int j = 0; j<nSample; j++)
				weightsigma += prob_cls[j][i] * (pnts[j]->z - nextMean[i])*(pnts[j]->z - nextMean[i]);
			nextSigma[i] = sqrt(weightsigma / ave);
		}

		if (fabs(mean[0] - nextMean[0])<0.00001)
			flg = false;
		else {
			for (int i = 0; i<nClass; i++) {
				mean[i] = nextMean[i];
				sigma[i] = nextSigma[i];
				alpha[i] = nextAlpha[i];
			}
		}
		nIter++;
		//	printf("%d th iteration, mean1:%0.3f sigma1:%0.3f alpha1:%0.3f mean2:%0.3f sigma2:%0.3f alpha2:%0.3f\n",nIter,mean[0],sigma[0],alpha[0],mean[1],sigma[1],alpha[1]);

		if (fabs(mean[0] - mean[1])< 0.005 || fabs(mean[1] - mean[2])<0.005)
			flg = false;

		if (nIter > 100) {
			flg = false;
		}
	}

	paras.clear();
	for (int i = 0; i<nClass; i++) {
		GMMPara para;
		para.mu = mean[i];
		para.sigma = sigma[i];
		para.alpha = alpha[i];
		paras.push_back(para);
	}

}

/**
 * @fn	void RailwayExtraction::ClusteringByGMM(vector<PointXYZDouble*> pnts, vector<GMMPara> paras, vector<vector<PointXYZDouble*>> &cPnts)
 *
 * @brief	Clustering using gmm parameters
 *
 * @author	Jaewook Jung
 * @date	10/27/2017
 *
 * @param			pnts 	If non-null, the input points.
 * @param 		  	paras	The input parameters.
 * @param [in,out]	cPnts	[in,out] If non-null, the clustered points.
 */

void RailwayExtraction::ClusteringByGMM(vector<PointXYZDouble*> pnts, vector<GMMPara> paras, vector<vector<PointXYZDouble*>> &cPnts)
{
	int nClass = paras.size();

	cPnts.clear();
	cPnts.resize(nClass);

	for (int i = 0; i<pnts.size(); i++) {
		vector<double> prob;
		for (int j = 0; j<nClass; j++)
			prob.push_back(0);

		for (int j = 0; j<nClass; j++) {
			prob[j] = 1 / (paras[j].sigma*sqrt(2 * PI))*exp(-1 * (pnts[i]->z - paras[j].mu)*(pnts[i]->z - paras[j].mu) / (2 * paras[j].sigma*paras[j].sigma)) * paras[j].alpha;
		}
		double maxVal = *max_element(prob.begin(), prob.end());
		int maxidx = -1;
		for (int j = 0; j<nClass; j++) {
			if (prob[j] == maxVal)
				maxidx = j;
		}
		if (maxidx == -1) {
			printf("clustering fail!! %d\n", pnts.size());
			return;
		}
		cPnts[maxidx].push_back(pnts[i]);
	}
}

/**
 * @fn	void RailwayExtraction::SegmentPoints(vector<PointXYZDouble*> Pnts, vector<PointXYZDouble*> &segPnts, double fDistTh)
 *
 * @brief	Segment points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param			Pnts   	If non-null, input points.
 * @param [in,out]	segPnts	[in,out] If non-null, segmented pnts.
 * @param 		  	fDistTh	The distance threshold.
 */

void RailwayExtraction::SegmentPoints(vector<PointXYZDouble*> Pnts, vector<PointXYZDouble*> &segPnts, double fDistTh)
{
	segPnts.clear();
	if (Pnts.size() == 0)
		return;

	for (int i = 0; i<Pnts.size(); i++)
		Pnts[i]->SegmentID = -1;

	int SegID = 0;
	vector<PtSegment> PtSegments;

	while (true)
	{
		PointXYZDouble* SeedPt = NULL;
		for (int i = 0; i<Pnts.size(); i++) {
			if (Pnts[i]->SegmentID < 0) {
				SeedPt = Pnts[i];
				break;
			}
		}
		if (SeedPt == NULL)
			break;

		PtSegment Segment;
		PointGrowing(SeedPt, Pnts, Segment, SegID, fDistTh);
		if (Segment.MemPts.size()> 100 || PtSegments.size() == 0)
			PtSegments.push_back(Segment);
		if (PtSegments.size()>1000)
			break;
	}

	sort(PtSegments.begin(), PtSegments.end(), LargePts);
	segPnts = PtSegments[0].MemPts;

	//	printf("# of Segment: %d \n",PtSegments.size());
	//	for(int i=0;i<PtSegments.size();i++)
	//		printf("# of MemPnts : %d \n",PtSegments[i].MemPts.size());


}

/**
 * @fn	void RailwayExtraction::PointGrowing(PointXYZDouble* pCurVh, vector<PointXYZDouble*> Pnts, PtSegment &Segment, int SegID, double fDistTh)
 *
 * @brief	Point growing
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pCurVh 	If non-null, a pointer of the current point.
 * @param			Pnts   	If non-null, input pnts.
 * @param [in,out]	Segment	segments.
 * @param 		  	SegID  	Segment Id.
 * @param 		  	fDistTh	The distance threshold.
 */

void RailwayExtraction::PointGrowing(PointXYZDouble* pCurVh, vector<PointXYZDouble*> Pnts, PtSegment &Segment, int SegID, double fDistTh)
{
	pCurVh->SegmentID = SegID;
	Segment.MemPts.push_back(pCurVh);

	for (int i = 0; i<Pnts.size(); i++) {
		if (Pnts[i] == pCurVh)
			continue;
		if (Pnts[i]->SegmentID >= 0)
			continue;
		double fdist = DistanceBetweenPoints2D(pCurVh, Pnts[i]);
		if (fdist < fDistTh)
			PointGrowing(Pnts[i], Pnts, Segment, SegID, fDistTh);
	}

}
void RailwayExtraction::GetMinMaxXY(vector<PointXYZDouble*> pnts, double &minX, double &maxX, double &minY, double &maxY)
{
	minX = 1e100, minY = 1e100, maxX = -1e100, maxY = -1e100;
	for (int i = 0; i<pnts.size(); i++) {
		if (minX>pnts[i]->x) minX = pnts[i]->x;
		if (maxX<pnts[i]->x) maxX = pnts[i]->x;
		if (minY>pnts[i]->y) minY = pnts[i]->y;
		if (maxY<pnts[i]->y) maxY = pnts[i]->y;
	}
}

int RailwayExtraction::ReCount(int totnum, int curnum)
{
	int newcount, multi;
	if (curnum >= totnum) {
		multi = curnum / totnum;
		newcount = curnum - totnum*multi;
	}
	else if (curnum <0)
		newcount = totnum + curnum;
	else
		newcount = curnum;

	return newcount;
}

/**
 * @fn	double RailwayExtraction::ComputeArea(vector<POINT3D> vtx)
 *
 * @brief	Calculates the area
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	vtx	The vertex.
 *
 * @return	The calculated area.
 */

double RailwayExtraction::ComputeArea(vector<POINT3D> vtx)
{
	double Area = 0;
	for (int i = 0; i<vtx.size(); i++) {
		int next = (i + 1) % vtx.size();
		POINT3D pnt = vtx[i];
		POINT3D npnt = vtx[next];

		Area = Area + (npnt.x - pnt.x)*(pnt.y + npnt.y);
	}
	return Area = 0.5*fabs(Area);
}

/**
 * @fn	double RailwayExtraction::ComputeInnerAngle(POINT3D sp, POINT3D mp, POINT3D ep)
 *
 * @brief	Calculates the inner angle
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	sp	The start point.
 * @param	mp	The middle point.
 * @param	ep	The end point.
 *
 * @return	The calculated inner angle.
 */

double RailwayExtraction::ComputeInnerAngle(POINT3D sp, POINT3D mp, POINT3D ep)
{
	double lx1 = sp.x - mp.x;
	double ly1 = sp.y - mp.y;
	double lx2 = ep.x - mp.x;
	double ly2 = ep.y - mp.y;

	double inner = (lx1*lx2 + ly1*ly2);

	double i1 = sqrt(lx1*lx1 + ly1*ly1);
	double i2 = sqrt(lx2*lx2 + ly2*ly2);

	lx1 = lx1 / i1;
	ly1 = ly1 / i1;
	lx2 = lx2 / i2;
	ly2 = ly2 / i2;

	inner = (lx1*lx2 + ly1*ly2);
	if (inner < -1)
		inner = -1;
	if (inner > 1)
		inner = 1;

	double result = acos(inner) * 180 / PI;

	//0~360
	/*	if(lx1>lx2){
	result = 360-result;
	}*/

	return result;
}

/**
 * @fn	POINT3D RailwayExtraction::ComputeIntersectionPoint3D(double A1, double B1, double C1, double A2, double B2, double C2)
 *
 * @brief	Calculates the intersection point 3D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	A1	The parameter of the first line (Ax+By+C=0).
 * @param	B1	The parameter of the first line.
 * @param	C1	The parameter of the first line.
 * @param	A2	The parameter of the second line.
 * @param	B2	The parameter of the second line.
 * @param	C2	The parameter of the second line.
 *
 * @return	The calculated intersection point 3D.
 */

POINT3D RailwayExtraction::ComputeIntersectionPoint3D(double A1, double B1, double C1, double A2, double B2, double C2)
{
	POINT3D pnt;

	if (B1*C2 - B2*C1 == 0)
		int tmp = 0;

	if (A1*B2 - B1*A2 == 0)
		int tmp = 0;

	pnt.x = (B1*C2 - B2*C1) / (A1*B2 - B1*A2);
	if (A2 != 1) {
		pnt.y = A2*pnt.x + C2;
		pnt.z = 0;
	}
	else {
		pnt.y = A1*pnt.x + C1;
		pnt.z = 0;
	}

	return pnt;
}

/**
 * @fn	POINT3D RailwayExtraction::ComputeFootOfPerpendicular(POINT3D p, Line3D ln)
 *
 * @brief	Calculates the foot of perpendicular
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	p 	A POINT3D to process.
 * @param	ln	The line.
 *
 * @return	The calculated foot of perpendicular.
 */

POINT3D RailwayExtraction::ComputeFootOfPerpendicular(POINT3D p, Line3D ln)
{
	POINT3D re;
	Line3D pLn;
	if (ln.A2D != 0 && ln.B2D != 0) {
		pLn.A2D = -1 / ln.A2D;
		pLn.B2D = -1;
		pLn.C2D = p.y - pLn.A2D * p.x;
		re.x = (pLn.C2D - ln.C2D) / (ln.A2D - pLn.A2D);
		re.y = ln.A2D * re.x + ln.C2D;
		re.z = (ln.BeginPt.z + ln.EndPt.z) / 2;
	}
	else if (ln.A2D == 0) {
		re.x = p.x;
		re.y = -ln.C2D / ln.B2D;
		re.z = (ln.BeginPt.z + ln.EndPt.z) / 2;
	}
	else {
		re.y = p.y;
		re.x = -ln.C2D / ln.A2D;
		re.z = (ln.BeginPt.z + ln.EndPt.z) / 2;
	}
	return re;
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPoints2D(POINT3D p1, POINT3D p2)
 *
 * @brief	Distance between points 2D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	p1	The first POINT3D.
 * @param	p2	The second POINT3D.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPoints2D(POINT3D p1, POINT3D p2)
{
	double dist = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
	return dist;
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPoints2D(PointXYZDouble* p1, PointXYZDouble* p2)
 *
 * @brief	Distance between points 2D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	p1	If non-null, the first PointXYZDouble.
 * @param [in,out]	p2	If non-null, the second PointXYZDouble.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPoints2D(PointXYZDouble* p1, PointXYZDouble* p2)
{
	double dist = sqrt((p1->x - p2->x)*(p1->x - p2->x) + (p1->y - p2->y)*(p1->y - p2->y));
	return dist;
}

double RailwayExtraction::DistanceBetweenPoints3D(PointXYZDouble* p1, PointXYZDouble* p2)
{
	double dist = sqrt((p1->x - p2->x)*(p1->x - p2->x) + (p1->y - p2->y)*(p1->y - p2->y) + (p1->z - p2->z)*(p1->z - p2->z));
	return dist;
}

double RailwayExtraction::DistanceBetweenPoints3D(POINT3D p1, POINT3D p2)
{
	double dist = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
	return dist;
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPointAndLine2D(POINT3D p, Line3D l)
 *
 * @brief	Distance between point and line 2D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	p	A POINT3D to process.
 * @param	l	A Line3D to process.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPointAndLine2D(POINT3D p, Line3D l)
{
	double dist = (l.A2D*p.x + l.B2D*p.y + l.C2D) / sqrt(l.A2D*l.A2D + l.B2D*l.B2D);
	return fabs(dist);
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPointAndLine2D(PointXYZDouble* p, Line3D l)
 *
 * @brief	Distance between point and line 2D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	p	If non-null, a PointXYZDouble to process.
 * @param 		  	l	A Line3D to process.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPointAndLine2D(PointXYZDouble* p, Line3D l)
{
	double dist = (l.A2D*p->x + l.B2D*p->y + l.C2D) / sqrt(l.A2D*l.A2D + l.B2D*l.B2D);
	return fabs(dist);
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPointAndLine3D(POINT3D _p, Line3D _ls)
 *
 * @brief	Distance between point and line 3D
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	_p 	A POINT3D to process.
 * @param	_ls	A Line3D to process.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPointAndLine3D(POINT3D _p, Line3D _ls)
{
	POINT3D midPt = (_ls.BeginPt + _ls.EndPt) / 2;
	double fProduct = _ls.m_UnitVec.x*(midPt.x - _p.x) + _ls.m_UnitVec.y*(midPt.y - _p.y) + _ls.m_UnitVec.z*(midPt.z - _p.z);
	double Dx = (midPt.x - _p.x) - fProduct*_ls.m_UnitVec.x;
	double Dy = (midPt.y - _p.y) - fProduct*_ls.m_UnitVec.y;
	double Dz = (midPt.z - _p.z) - fProduct*_ls.m_UnitVec.z;
	double fDist = sqrt(Dx*Dx + Dy*Dy + Dz*Dz);

	return fDist;
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPointAndLine3DHgt(PointXYZDouble* _p, Line3D _ls)
 *
 * @brief	Height Difference between point and line 3D 
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	_p 	If non-null, a PointXYZDouble to process.
 * @param 		  	_ls	A Line3D to process.
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPointAndLine3DHgt(PointXYZDouble* _p, Line3D _ls)
{
	POINT3D midPt = (_ls.BeginPt + _ls.EndPt) / 2;
	double fProduct = _ls.m_UnitVec.x*(midPt.x - _p->x) + _ls.m_UnitVec.y*(midPt.y - _p->y) + _ls.m_UnitVec.z*(midPt.z - _p->z);
	double Dx = (midPt.x - _p->x) - fProduct*_ls.m_UnitVec.x;
	double Dy = (midPt.y - _p->y) - fProduct*_ls.m_UnitVec.y;
	double Dz = (midPt.z - _p->z) - fProduct*_ls.m_UnitVec.z;
	double fDist = sqrt(Dz*Dz);

	return fDist;
}

/**
 * @fn	double RailwayExtraction::DistanceBetweenPointAndPlane(POINT3D p, PlnPara para)
 *
 * @brief	Distance between point and plane
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	p   	A POINT3D to process.
 * @param	para	The plane parameter (Ax+By+Cz+D=0).
 *
 * @return	A double.
 */

double RailwayExtraction::DistanceBetweenPointAndPlane(POINT3D p, PlnPara para)
{
	double dist = (p.x*para.a + p.y*para.b + p.z*para.c + para.d) / sqrt(para.a*para.a + para.b*para.b + para.c*para.c);
	return fabs(dist);
}

/**
 * @fn	vector<int> RailwayExtraction::GetNeighborIdx(double _x, double _y, int _NumX, int _NumY, double _gSize, int _nSize)
 *
 * @brief	Gets neighbor index
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	_x	  	The x coordinate.
 * @param	_y	  	The y coordinate.
 * @param	_NumX 	Number of grids in x.
 * @param	_NumY 	Number of grids in y.
 * @param	_gSize	The grid size.
 * @param	_nSize	The number of neighbors.
 *
 * @return	The neighbor index.
 */

vector<int> RailwayExtraction::GetNeighborIdx(double _x, double _y, int _NumX, int _NumY, double _gSize, int _nSize)
{
	int x, y, idx;
	x = floor((_x - m_BoundMin.x) / (double)_gSize);
	y = floor((_y - m_BoundMin.y) / (double)_gSize);

	int neighbor = (int)_nSize / 2;
	vector<int> list;
	for (int i = 0; i<_nSize; i++) {
		for (int j = 0; j<_nSize; j++) {
			int r = x + i - neighbor;
			int c = y + j - neighbor;

			if (r < 0 || r >= _NumX)
				idx = -1;
			else if (c < 0 || c >= _NumY)
				idx = -1;
			else
				idx = c*_NumX + r;
			list.push_back(idx);
		}
	}
	return list;
}

/**
 * @fn	int RailwayExtraction::GetDirectionMode(double nx, double ny, double px, double py)
 *
 * @brief	Gets direction mode
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	nx	x coordinate of the second point.
 * @param	ny	y coordinate of the second point.
 * @param	px	x coordinate of the first point.
 * @param	py	y coordinate of the first point.
 *
 * @return	The direction mode.
 */

int RailwayExtraction::GetDirectionMode(double nx, double ny, double px, double py)
{
	double dX = nx - px;
	double dY = ny - py;

	if (dX >= 0 && dY >= 0)
		return 1;
	else if (dX < 0 && dY >= 0)
		return 2;
	else if (dX < 0 && dY < 0)
		return 3;
	else
		return 4;

}

/**
 * @fn	void RailwayExtraction::KalmanPredict(int mode, double ang, double delL, Matrix<double> curPt, Matrix<double> curP, Matrix<double> &nextPt, Matrix<double> &nextP)
 *
 * @brief	Kalman predict
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	mode  	The mode.
 * @param 		  	ang   	The angle.
 * @param 		  	delL  	The length of line.
 * @param 		  	curPt 	The current point.
 * @param 		  	curP  	The current weigth.
 * @param [in,out]	nextPt	The next point.
 * @param [in,out]	nextP 	The next weight.
 */

void RailwayExtraction::KalmanPredict(int mode, double ang, double delL, Matrix<double> curPt, Matrix<double> curP, Matrix<double> &nextPt, Matrix<double> &nextP)
{
	int nX = 3;

	Matrix<double> A(nX, nX, 0);
	Matrix<double> Q(nX, nX, 0);

	//	Matrix<double> predPt(nX,1,0);
	//	Matrix<double> predP(nX,nX,0);

	ang = ang * PI / 180;

	if (mode == 1) {
		A(0, 0) = 1 + 1 / curPt(0, 0)*fabs(delL*cos(ang));
		A(1, 1) = 1 + 1 / curPt(1, 0)*fabs(delL*sin(ang));
		A(2, 2) = 1;
	}
	else if (mode == 2) {
		A(0, 0) = 1 - 1 / curPt(0, 0)*fabs(delL*cos(ang));
		A(1, 1) = 1 + 1 / curPt(1, 0)*fabs(delL*sin(ang));
		A(2, 2) = 1;
	}
	else if (mode == 3) {
		A(0, 0) = 1 - 1 / curPt(0, 0)*fabs(delL*cos(ang));
		A(1, 1) = 1 - 1 / curPt(1, 0)*fabs(delL*sin(ang));
		A(2, 2) = 1;
	}
	else {
		A(0, 0) = 1 + 1 / curPt(0, 0)*fabs(delL*cos(ang));
		A(1, 1) = 1 - 1 / curPt(1, 0)*fabs(delL*sin(ang));
		A(2, 2) = 1;
	}


	Q(0, 0) = 0.005;	//(0.0005) if value is large, observation affect more
	Q(1, 1) = 0.005;
	Q(2, 2) = 0.01;


	nextPt = A*curPt;
	nextP = A*curP*A.Transpose() + Q;

}

/**
 * @fn	void RailwayExtraction::KalmanUpdate(Matrix<double> predPt, Matrix<double> predP, Matrix<double> obsPt, double sigma, Matrix<double> &hatPt, Matrix<double> &hatP)
 *
 * @brief	Kalman update
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	predPt	The predicated point.
 * @param 		  	predP 	The predicated weight.
 * @param 		  	obsPt 	The observation matrix.
 * @param 		  	sigma 	The sigma.
 * @param [in,out]	hatPt 	The updated point.
 * @param [in,out]	hatP  	The updated weight.
 */

void RailwayExtraction::KalmanUpdate(Matrix<double> predPt, Matrix<double> predP, Matrix<double> obsPt, double sigma, Matrix<double> &hatPt, Matrix<double> &hatP)
{
	int nX = 3;
	Matrix<double> H(nX, nX, 0);
	Matrix<double> R(nX, nX, 0);
	Matrix<double> I(nX, nX, 0);

	for (int i = 0; i<nX; i++) {
		I(i, i) = 1;
		H(i, i) = 1;
		R(i, i) = sigma;
	}

	Matrix<double> PHT = predP*H.Transpose();
	Matrix<double> HPHT = H*predP*H.Transpose();
	Matrix<double> K = PHT*(HPHT + R).Inverse();
	hatPt = predPt + K*(obsPt - H*predPt);
	hatP = (I - K*H)*predP;

}

/**
 * @fn	void RailwayExtraction::GetPntsInsidePolygon(vector<POINT3D> inVtx, vector<PointXYZDouble*> inPts, vector<PointXYZDouble*> &outPts)
 *
 * @brief	Gets pnts inside polygon
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	inVtx 	The vertices of a polygon.
 * @param [in,out]	inPts 	If non-null, input points.
 * @param [in,out]	outPts	[in,out] If non-null, output points.
 */

void RailwayExtraction::GetPntsInsidePolygon(vector<POINT3D> inVtx, vector<PointXYZDouble*> inPts, vector<PointXYZDouble*> &outPts)
{
	outPts.clear();
	vector<double> vX, vY;
	for (int i = 0; i<inVtx.size(); i++) {
		vX.push_back(inVtx[i].x);
		vY.push_back(inVtx[i].y);
	}
	for (int i = 0; i<inPts.size(); i++) {
		if (pnpoly2(vX.size(), &(vX.front()), &(vY.front()), inPts[i]->x, inPts[i]->y) == true) {
			outPts.push_back(inPts[i]);
		}
	}
}

/**
 * @fn	void RailwayExtraction::ExtractPlaneByRANSAC(vector<PointXYZDouble*> inPts, PlnPara &para, double ThDist, vector<PointXYZDouble*> &sptPnts)
 *
 * @brief	Extracts the plane by ransac
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	inPts  	If non-null, input points.
 * @param [in,out]	para   	The plane parameters.
 * @param 		  	ThDist 	The distance threshold.
 * @param [in,out]	sptPnts	[in,out] If non-null, the supporting pnts.
 */

void RailwayExtraction::ExtractPlaneByRANSAC(vector<PointXYZDouble*> inPts, PlnPara &para, double ThDist, vector<PointXYZDouble*> &sptPnts)
{
	int MaxIter = 100;
	int nCnt = inPts.size();
	if (nCnt < 4) {
		return;
	}

	sptPnts.clear();
	int MaxCnt = 0;
	for (int i = 0; i<MaxIter; i++) {
		int idx1 = rand() % inPts.size();
		int idx2 = rand() % inPts.size();
		int idx3 = rand() % inPts.size();

		POINT3D pt1, pt2, pt3;
		pt1.SetPoint3D(inPts[idx1]->x, inPts[idx1]->y, inPts[idx1]->z);
		pt2.SetPoint3D(inPts[idx2]->x, inPts[idx2]->y, inPts[idx2]->z);
		pt3.SetPoint3D(inPts[idx3]->x, inPts[idx3]->y, inPts[idx3]->z);

		PlnPara tmpPara = GetPlaneParameters3p(pt1, pt2, pt3);

		vector<PointXYZDouble*> tmptmpPnts;
		for (int j = 0; j<inPts.size(); j++) {
			POINT3D pt;
			pt.x = inPts[j]->x; pt.y = inPts[j]->y; pt.z = inPts[j]->z;
			double fdist = DistanceBetweenPointAndPlane(pt, tmpPara);
			if (fdist < ThDist) {
				tmptmpPnts.push_back(inPts[j]);
			}
		}
		tmpPara = GetPlaneParametersEigen(tmptmpPnts);
		tmptmpPnts.clear();
		int count = 0;
		for (int j = 0; j<inPts.size(); j++) {
			POINT3D pt;
			pt.x = inPts[j]->x; pt.y = inPts[j]->y; pt.z = inPts[j]->z;
			double fdist = DistanceBetweenPointAndPlane(pt, tmpPara);
			if (fdist < ThDist) {
				count++;
				tmptmpPnts.push_back(inPts[j]);
			}
		}

		if (count > MaxCnt) {
			MaxCnt = count;
			sptPnts = tmptmpPnts;
			para = tmpPara;
		}
	}


	//	printf("%d %0.4f %0.4f %0.4f %0.4f\n",sptPnts.size(), para.a, para.b, para.c, para.d);

}

/**
 * @fn	PlnPara RailwayExtraction::GetPlaneParameters3p(POINT3D p1, POINT3D p2, POINT3D p3)
 *
 * @brief	Gets plane parameters with 3 points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	p1	The first POINT3D.
 * @param	p2	The second POINT3D.
 * @param	p3	The third POINT3D.
 *
 * @return	The plane parameters.
 */

PlnPara RailwayExtraction::GetPlaneParameters3p(POINT3D p1, POINT3D p2, POINT3D p3)
{
	PlnPara para;
	POINT3D BA, CB, ABBC;

	BA.x = p2.x - p1.x;
	BA.y = p2.y - p1.y;
	BA.z = p2.z - p1.z;

	CB.x = p3.x - p2.x;
	CB.y = p3.y - p2.y;
	CB.z = p3.z - p2.z;

	ABBC.x = BA.y*CB.z - CB.y*BA.z;
	ABBC.y = -1 * (BA.x*CB.z - CB.x*BA.z);
	ABBC.z = BA.x*CB.y - CB.x*BA.y;

	para.a = -1 * ABBC.x;
	para.b = -1 * ABBC.y;
	para.c = -1 * ABBC.z;
	para.d = p2.x*ABBC.x + p2.y*ABBC.y + p2.z*ABBC.z;

	return para;
}

/**
 * @fn	PlnPara RailwayExtraction::GetPlaneParametersEigen(vector<PointXYZDouble*> pts)
 *
 * @brief	Gets plane parameters with more than three points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pts	If non-null, input points.
 *
 * @return	The plane parameters eigen.
 */

PlnPara RailwayExtraction::GetPlaneParametersEigen(vector<PointXYZDouble*> pts)
{
	PlnPara para;
	POINT3D centerPt(0, 0, 0);
	int i;
	int n = pts.size();
	POINT3D t;

	for (i = 0; i<n; i++) {
		centerPt.x = (i*centerPt.x + pts[i]->x) / (double)(i + 1);
		centerPt.y = (i*centerPt.y + pts[i]->y) / (double)(i + 1);
		centerPt.z = (i*centerPt.z + pts[i]->z) / (double)(i + 1);
	}

	TNT::Array2D<double> M_A(3, 3, 0.0);

	for (i = 0; i<n; i++) {
		M_A[0][0] += (pts[i]->x - centerPt.x)*(pts[i]->x - centerPt.x);
		M_A[0][1] += (pts[i]->x - centerPt.x)*(pts[i]->y - centerPt.y);
		M_A[0][2] += (pts[i]->x - centerPt.x)*(pts[i]->z - centerPt.z);
		M_A[1][0] += (pts[i]->x - centerPt.x)*(pts[i]->y - centerPt.y);
		M_A[1][1] += (pts[i]->y - centerPt.y)*(pts[i]->y - centerPt.y);
		M_A[1][2] += (pts[i]->y - centerPt.y)*(pts[i]->z - centerPt.z);
		M_A[2][0] += (pts[i]->x - centerPt.x)*(pts[i]->z - centerPt.z);
		M_A[2][1] += (pts[i]->y - centerPt.y)*(pts[i]->z - centerPt.z);
		M_A[2][2] += (pts[i]->z - centerPt.z)*(pts[i]->z - centerPt.z);
	}


	JAMA::Eigenvalue<double> eig(M_A);
	TNT::Array2D<double> eigenVec;
	eig.getV(eigenVec);

	para.a = eigenVec[0][0];
	para.b = eigenVec[1][0];
	para.c = eigenVec[2][0];

	para.d = -1 * (para.a*centerPt.x + para.b*centerPt.y + para.c*centerPt.z);

	return para;
}

/**
 * @fn	void RailwayExtraction::GetLnsFromVtx(vector<POINT3D> pts, vector<Line3D> &Lns)
 *
 * @brief	Gets lines from vertex
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param 		  	pts	The input vertices.
 * @param [in,out]	Lns	The out lines.
 */

void RailwayExtraction::GetLnsFromVtx(vector<POINT3D> pts, vector<Line3D> &Lns)
{
	Lns.clear();
	for (int i = 0; i<pts.size(); i++) {
		int next = ReCount(pts.size(), i + 1);
		Line3D Ln;
		Ln.BeginPt = pts[i];
		Ln.EndPt = pts[next];
		Lns.push_back(Ln);
	}
}

/**
 * @fn	void RailwayExtraction::PrintTurnoutType(int type)
 *
 * @brief	Print turnout type
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	type	The type.
 */

void RailwayExtraction::PrintTurnoutType(int type)
{
	if (type == 0)
		printf("Split to right(open, start)\n");
	else if (type == 1)
		printf("Split to left (open, start)\n");
	else if (type == 2)
		printf("Merge from right (start)\n");
	else if (type == 3)
		printf("Merge from left (start)\n");
	else if (type == 4)
		printf("Split to right (close, start)\n");
	else if (type == 5)
		printf("Split to left (close, start)\n");
	else if (type == 6)
		printf("Merge from right (open, end)\n");
	else if (type == 7)
		printf("Merge from left(open, end)\n");
	else if (type == 8)
		printf("Split to right (end)\n");
	else if (type == 9)
		printf("Split to left (end)\n");
	else if (type == 10)
		printf("Merge from right (close, end)\n");
	else if (type == 11)
		printf("Merge from left (close, end)\n");
	else if (type == 12)
		printf("Split to left (both, start)\n");
	else if (type == 13)
		printf("Split to right (both, start)\n");
	else if (type == 14)
		printf("Merge from right (both, end)\n");
	else if (type == 15)
		printf("Merge from left (both, end)\n");
	else if (type == 16)
		printf("Merge from left (start)\n");
	else if (type == 17)
		printf("Merge from right (start)\n");
	else if (type == 18)
		printf("Split to right (end)\n");
	else if (type == 19)
		printf("Split to left (end)\n");
	else
		printf("not defined\n");
}

/**
 * @fn	void RailwayExtraction::PrintVtx(vector<POINT3D> vtx)
 *
 * @brief	Print vertex
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	vtx	input vertices.
 */

void RailwayExtraction::PrintVtx(vector<POINT3D> vtx)
{
	for (int i = 0; i<vtx.size(); i++) {
		int next = ReCount(vtx.size(), i + 1);
		POINT3D p1 = vtx[i];
		POINT3D p2 = vtx[next];
		printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);

	}
}

/**
 * @fn	void RailwayExtraction::PrintPts(vector<PointXYZDouble*> pts)
 *
 * @brief	Print points
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param [in,out]	pts	If non-null, input points.
 */

void RailwayExtraction::PrintPts(vector<PointXYZDouble*> pts)
{
	for (int i = 0; i<pts.size(); i++) {
		printf("%0.3f %0.3f %0.3f\n", pts[i]->x, pts[i]->y, pts[i]->z);
	}
}

/**
 * @fn	void RailwayExtraction::PrintLines(vector<Line3D> lns)
 *
 * @brief	Print lines
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	lns	input lines.
 */

void RailwayExtraction::PrintLines(vector<Line3D> lns)
{
	for (int i = 0; i<lns.size(); i++) {
		POINT3D p1 = lns[i].BeginPt;
		POINT3D p2 = lns[i].EndPt;
		printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}
}

/**
 * @fn	void RailwayExtraction::updateView(int ClassView)
 *
 * @brief	Updates the view described by ClassView
 *
 * @author	Jaewook Jung
 * @date	10/26/2017
 *
 * @param	ClassView	The class view.
 */

void RailwayExtraction::updateView(int ClassView)
{
	updateView(ClassView, nullptr);
}

void  RailwayExtraction::updateView(int ClassView, TreeNodeCollection ^collection)
{
	clock_t tStart = clock();

	this->windowController->RemoveActors();
	//this->windowController->ReloadView();

	if (points == NULL)
	{
		points = vtkSmartPointer<vtkPoints>::New();
		points->SetDataTypeToDouble();

		depthArray = vtkSmartPointer<vtkDoubleArray>::New();
		depthArray->SetName("DepthArray");

		intensityArray = vtkSmartPointer<vtkDoubleArray>::New();
		intensityArray->SetName("IntensityArray");

		minIntensity = point_cloud->points[0].Intensity;
		maxIntensity = minIntensity;
	
		minDepth = point_cloud->points[0].z;
		maxDepth = minDepth;

		cell = vtkSmartPointer<vtkCellArray>::New();

		for (size_t i = 0; i < point_cloud->points.size(); i++)
		{
			double x = point_cloud->points[i].x;
			double y = point_cloud->points[i].y;
			double z = point_cloud->points[i].z;

			double point[3] = { x, y, z };
			vtkIdType id = points->InsertNextPoint(point);

			cell->InsertNextCell(1);
			cell->InsertCellPoint(id);

			double intensity = point_cloud->points[i].Intensity;
			intensityArray->InsertNextValue(intensity);

			if (minIntensity > intensity) minIntensity = intensity;
			if (maxIntensity < intensity) maxIntensity = intensity;

			double depth = point_cloud->points[i].z;
			depthArray->InsertNextValue(depth);

			if (minDepth > depth) minDepth = depth;
			if (maxDepth < depth) maxDepth = depth;
		}
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

	polyData->SetPoints(points);
	polyData->SetVerts(cell);

	if (ClassView == 2)
	{
		polyData->GetPointData()->SetScalars(intensityArray);
		polyData->GetPointData()->SetActiveScalars("IntensityArray");
	}
	else if (ClassView == 3)
	{
		polyData->GetPointData()->SetScalars(depthArray);
		polyData->GetPointData()->SetActiveScalars("DepthArray");
	}

	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter = vtkSmartPointer<vtkCenterOfMass>::New();

	centerOfMassFilter->SetInputData(polyData);
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();

	double center[3];
	centerOfMassFilter->GetCenter(center);

	SharedData::getInstance()->centerOfMass[0] = center[0];
	SharedData::getInstance()->centerOfMass[1] = center[1];
	SharedData::getInstance()->centerOfMass[2] = center[2];

	vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
	translation->Translate(-1 * center[0], -1 * center[1], -1 * center[2]);

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(polyData);
	transformFilter->SetTransform(translation);
	transformFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(polyData);
	mapper->SetInputConnection(transformFilter->GetOutputPort());
	mapper->SetColorModeToDefault();

	if (ClassView == 2)
	{
		mapper->SetScalarRange(minIntensity, maxIntensity);
	}
	else if (ClassView == 3)
	{
		mapper->SetScalarRange(minDepth, maxDepth);
	}

	mapper->SetScalarVisibility(1);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	//vtkSmartPointer<vtkActorCollection> actors = vtkSmartPointer<vtkActorCollection>::New();
	//actors->AddItem(actor);


	bool showRail = true;
	bool showBND = true;
	bool showTurnOut = true;
	bool showLoopCross = true;
	bool showToePoints = true;
	bool showSwitchRegions = true;

	if (ClassView == 1) {
		
		actor->GetProperty()->SetColor(this->defaultColor_R / 255.0, this->defaultColor_G / 255.0, this->defaultColor_B / 255.0);

		//Rail
		if (showRail)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];

				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					
					display = node->Nodes[0]->Checked;

					if (!node->Checked) 
					{
						display = false;
					}
				}
				
				if (display)
				{
					if (layer->m_RailVecCenter.size() > 0) {
						for (int i = 0; i < layer->m_RailVecCenter.size() - 1; i++) {
							double p0[3] = { layer->m_RailVecCenter[i].x,layer->m_RailVecCenter[i].y,layer->m_RailVecCenter[i].z };
							double p1[3] = { layer->m_RailVecCenter[i + 1].x,layer->m_RailVecCenter[i + 1].y,layer->m_RailVecCenter[i + 1].z };

							vtkIdType point0 = pts->InsertNextPoint(p0);
							vtkIdType point1 = pts->InsertNextPoint(p1);

							vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
							line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
							line0->GetPointIds()->SetId(1, point1);
							lines->InsertNextCell(line0);
						}
					}				
				}

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];

					display = node->Nodes[4]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{
					if (layer->m_RailVec1.size() > 0) {
						for (int i = 0; i < layer->m_RailVec1.size() - 1; i++) {

							double p0[3] = { layer->m_RailVec1[i].x, layer->m_RailVec1[i].y, layer->m_RailVec1[i].z };
							double p1[3] = { layer->m_RailVec1[i + 1].x, layer->m_RailVec1[i + 1].y, layer->m_RailVec1[i + 1].z };

							vtkIdType point0 = pts->InsertNextPoint(p0);
							vtkIdType point1 = pts->InsertNextPoint(p1);

							vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
							line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
							line0->GetPointIds()->SetId(1, point1);
							lines->InsertNextCell(line0);
						}
					}
					
					if (layer->m_RailVec2.size() > 0) {
						for (int i = 0; i < layer->m_RailVec2.size() - 1; i++) {
							double p0[3] = { layer->m_RailVec2[i].x,layer->m_RailVec2[i].y,layer->m_RailVec2[i].z };
							double p1[3] = { layer->m_RailVec2[i + 1].x,layer->m_RailVec2[i + 1].y,layer->m_RailVec2[i + 1].z };

							vtkIdType point0 = pts->InsertNextPoint(p0);
							vtkIdType point1 = pts->InsertNextPoint(p1);

							vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
							line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
							line0->GetPointIds()->SetId(1, point1);
							lines->InsertNextCell(line0);
						}
					}
				}
			}

			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}

		//updated BNDs
		if (showBND)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];

				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					display = node->Nodes[4]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{

					for (int i = 0; i < layer->m_updateBnd.size(); i++) {
						double p0[3] = { layer->m_updateBnd[i].BeginPt.x,layer->m_updateBnd[i].BeginPt.y,layer->m_updateBnd[i].BeginPt.z };
						double p1[3] = { layer->m_updateBnd[i].EndPt.x,layer->m_updateBnd[i].EndPt.y,layer->m_updateBnd[i].EndPt.z };

						vtkIdType point0 = pts->InsertNextPoint(p0);
						vtkIdType point1 = pts->InsertNextPoint(p1);

						vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
						line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
						line0->GetPointIds()->SetId(1, point1);
						lines->InsertNextCell(line0);
					}
				}

			}
			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			thisActor->GetProperty()->SetColor(0, 0, 0);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}

		//Highlight turnout regions
		if (showTurnOut)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];

				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					display = node->Nodes[4]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{

					for (int i = 0; i < layer->m_updateBnd_turnout.size(); i++) {
						double p0[3] = { layer->m_updateBnd_turnout[i].BeginPt.x,layer->m_updateBnd_turnout[i].BeginPt.y,layer->m_updateBnd_turnout[i].BeginPt.z };
						double p1[3] = { layer->m_updateBnd_turnout[i].EndPt.x,layer->m_updateBnd_turnout[i].EndPt.y,layer->m_updateBnd_turnout[i].EndPt.z };

						vtkIdType point0 = pts->InsertNextPoint(p0);
						vtkIdType point1 = pts->InsertNextPoint(p1);

						vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
						line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
						line0->GetPointIds()->SetId(1, point1);
						lines->InsertNextCell(line0);
					}
				}
			}

			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			thisActor->GetProperty()->SetColor(255 / 255.0 , 0, 0);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}

		//Loop cross
		if (showLoopCross)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];

				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					display = node->Nodes[1]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{

					for (int i = 0; i < layer->m_LoopCross.size(); i++) {
						double p0[3] = { layer->m_LoopCross[i].lc.BeginPt.x,layer->m_LoopCross[i].lc.BeginPt.y,layer->m_LoopCross[i].lc.BeginPt.z };
						double p1[3] = { layer->m_LoopCross[i].lc.EndPt.x,layer->m_LoopCross[i].lc.EndPt.y,layer->m_LoopCross[i].lc.EndPt.z };

						vtkIdType point0 = pts->InsertNextPoint(p0);
						vtkIdType point1 = pts->InsertNextPoint(p1);

						vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
						line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
						line0->GetPointIds()->SetId(1, point1);
						lines->InsertNextCell(line0);
					}
				}
			}

			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			thisActor->GetProperty()->SetColor(0, 255 / 255.0, 0);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}

		//Toe points (Line)
		if (showToePoints)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];
				
				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					display = node->Nodes[3]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{

					for (int i = 0; i < layer->m_ToePts.size(); i++) {
						double p0[3] = { layer->m_ToePts[i].LeftPt.x,layer->m_ToePts[i].LeftPt.y,layer->m_ToePts[i].LeftPt.z };
						double p1[3] = { layer->m_ToePts[i].RightPt.x,layer->m_ToePts[i].RightPt.y,layer->m_ToePts[i].RightPt.z };

						vtkIdType point0 = pts->InsertNextPoint(p0);
						vtkIdType point1 = pts->InsertNextPoint(p1);

						vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
						line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
						line0->GetPointIds()->SetId(1, point1);
						lines->InsertNextCell(line0);
					}
				}
			}

			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			thisActor->GetProperty()->SetColor(255 / 255.0, 0, 255 / 255.0);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}

		//Switch regions
		if (showSwitchRegions)
		{
			vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
			vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
			pts->SetDataTypeToDouble();

			vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

			for (int l = 0; l < m_trackLayers.size(); l++)
			{
				TrackLayer *layer = m_trackLayers[l];

				bool display = true;

				display = collection == nullptr;

				if (collection != nullptr)
				{
					TreeNode ^node = collection[l];
					display = node->Nodes[2]->Checked;

					if (!node->Checked)
					{
						display = false;
					}
				}

				if (display)
				{

					for (int i = 0; i < layer->m_SwitchBoxes.size(); i++) {
						for (int j = 0; j < layer->m_SwitchBoxes[i].rgn.size(); j++) {
							int next = ReCount(layer->m_SwitchBoxes[i].rgn.size(), j + 1);
							double p0[3] = { layer->m_SwitchBoxes[i].rgn[j].x,layer->m_SwitchBoxes[i].rgn[j].y,layer->m_SwitchBoxes[i].rgn[j].z };
							double p1[3] = { layer->m_SwitchBoxes[i].rgn[next].x,layer->m_SwitchBoxes[i].rgn[next].y,layer->m_SwitchBoxes[i].rgn[next].z };

							vtkIdType point0 = pts->InsertNextPoint(p0);
							vtkIdType point1 = pts->InsertNextPoint(p1);

							vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
							line0->GetPointIds()->SetId(0, point0); // the second 0 is the index of the Origin in linesPolyData's points
							line0->GetPointIds()->SetId(1, point1);
							lines->InsertNextCell(line0);
						}
					}
				}
			}

			linesPolyData->SetPoints(pts);
			linesPolyData->SetLines(lines);

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(linesPolyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(transformFilter->GetOutputPort());
			mapper->SetColorModeToDefault();
			mapper->SetScalarVisibility(1);

			vtkSmartPointer<vtkActor> thisActor = vtkSmartPointer<vtkActor>::New();
			thisActor->SetMapper(mapper);
			thisActor->GetProperty()->SetLineWidth(5);
			thisActor->GetProperty()->SetColor(255 / 255.0, 0, 255 / 255.0);
			//actors->AddItem(thisActor);
			this->windowController->AddActorWithoutReloading(thisActor);
		}
	}

	if (ClassView == 3)
	{
		this->windowController->AddActor(actor);
	}
	else
	{
		this->windowController->AddActorWithoutReloading(actor);
		this->windowController->ReloadView();
	}	

	printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

std::vector<TrackLayer *> RailwayExtraction::GetLayers()
{
	return m_trackLayers;
}

vector<GradeSect> RailwayExtraction::ComputeGrade(TrackLayer *layer)
{
	double ThDPv = 1;

	//Divide Grade Section 
	DetermineGradeSection(layer->m_RailVecCenter, ThDPv);

	//Compute Grade parameters
	vector<GradeSect> grdsects;
	ComputeGradeParameters(layer->m_RailVecCenter, grdsects);

	//Print grade report
	if (grdsects.size() > 0) {
		//first line
		printf("# of Grade Sections: %d\n", grdsects.size());
		printf("X,Y,Z,Distance from Start,Grade Begin,Grade End,Grade Difference\n");
		printf("%0.2f,%0.2f,%0.2f,0.00,-,%0.2f,%0.2f\n"
			, grdsects[0].BeginPt.x, grdsects[0].BeginPt.y, grdsects[0].BeginPt.z
			, grdsects[0].grade, grdsects[0].grade);

		double totLength = grdsects[0].arc;

		//middle lines
		for (int i = 1; i < grdsects.size(); i++) {
			printf("%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n"
				, grdsects[i].BeginPt.x, grdsects[i].BeginPt.y, grdsects[i].BeginPt.z
				, totLength, grdsects[i - 1].grade, grdsects[i].grade, grdsects[i].grade - grdsects[i - 1].grade);

			totLength = totLength + grdsects[i].arc;
		}
		//last line 
		printf("%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,-,%0.2f\n"
			, grdsects[grdsects.size() - 1].EndPt.x, grdsects[grdsects.size() - 1].EndPt.y
			, grdsects[grdsects.size() - 1].EndPt.z, totLength
			, grdsects[grdsects.size() - 1].grade, -1 * grdsects[grdsects.size() - 1].grade);
	}

	return grdsects;
}

void RailwayExtraction::DetermineGradeSection(vector<POINT3D> &railVtx, double thDPv)
{
	if (railVtx.size() == 0)
		return;

	//divde rail vectors (raileVtx) into sections
	railVtx[0].PtType = 1;
	railVtx[railVtx.size() - 1].PtType = 1;
	DouglasPeuckerVertical(railVtx, 0, railVtx.size() - 1, thDPv);

}

void RailwayExtraction::DouglasPeuckerVertical(vector<POINT3D> &railVtx, int _sIdx, int _eIdx, double thDP)
{
	if (_eIdx <= _sIdx)
		return;

	//exchange Y and Z axis to consider Z value (Vertical)
	vector<POINT3D> tmpVtx;
	for (int i = 0; i < railVtx.size(); i++) {
		POINT3D pt;
		pt.x = railVtx[i].x;
		pt.y = railVtx[i].z;
		pt.z = railVtx[i].y;
		tmpVtx.push_back(pt);
	}

	Line3D ln;
	ln.BeginPt = tmpVtx[_sIdx];
	ln.EndPt = tmpVtx[_eIdx];
	ln.GetLineParameter();

	double fmax = -1e100;
	int vid = -1;

	for (int i = _sIdx + 1; i < _eIdx; i++) {
		POINT3D pt = tmpVtx[i];
		double dist = DistanceBetweenPointAndLine2D(pt, ln);
		if (dist > fmax) {
			fmax = dist;
			vid = i;
		}
	}

	if (vid > 0 && fmax > thDP) {
		railVtx[vid].PtType = 1;	//PtType == 1 for grade vertex 
		DouglasPeuckerVertical(railVtx, _sIdx, vid, thDP);
		DouglasPeuckerVertical(railVtx, vid, _eIdx, thDP);
	}
	else
		return;
}

void RailwayExtraction::ComputeGradeParameters(vector<POINT3D> railVtx, vector<GradeSect> &grdsects)
{
	grdsects.clear();
	GradeSect tmpGrdSect;
	for (int i = 0; i < railVtx.size(); i++) {
		if (railVtx[i].PtType != 1) {
			tmpGrdSect.MemVtx.push_back(railVtx[i]);
		}
		else {
			tmpGrdSect.MemVtx.push_back(railVtx[i]);
			if(tmpGrdSect.MemVtx.size() > 1)
				grdsects.push_back(tmpGrdSect);
			tmpGrdSect.ResetGradeSect();
			tmpGrdSect.MemVtx.push_back(railVtx[i]);
		}
	}
	if(tmpGrdSect.MemVtx.size() > 1)
		grdsects.push_back(tmpGrdSect);
	


	//Set values
	for (int i = 0; i < grdsects.size(); i++) {
		//set begin/end points
		grdsects[i].BeginPt = grdsects[i].MemVtx[0];
		grdsects[i].EndPt = grdsects[i].MemVtx[grdsects[i].MemVtx.size() - 1];

		//compute chord
		grdsects[i].chord = DistanceBetweenPoints2D(grdsects[i].BeginPt, grdsects[i].EndPt);
		Line3D chordLn;
		chordLn.BeginPt = grdsects[i].BeginPt;
		chordLn.EndPt = grdsects[i].EndPt;
		chordLn.GetLineParameter();

		//compute arc and maxh using member vertex
		double maxh = -1e100;	//tmp variable for calculating radius
		double length = 0;
		for (int j = 0; j < grdsects[i].MemVtx.size() - 1; j++) {
			POINT3D sp = grdsects[i].MemVtx[j];
			POINT3D ep = grdsects[i].MemVtx[j + 1];
			length = length + DistanceBetweenPoints2D(sp, ep);
			
			double fdist = DistanceBetweenPointAndLine2D(sp, chordLn);
			if (maxh < fdist) 
				maxh = fdist;
		}
		grdsects[i].arc = length;

		//compute height, grade, radius, degree of curve
		grdsects[i].height = grdsects[i].EndPt.z - grdsects[i].BeginPt.z;
		grdsects[i].grade = grdsects[i].height / grdsects[i].arc * 100;
		grdsects[i].radius = grdsects[i].chord * grdsects[i].chord - 8 * maxh + 4 * maxh * maxh;
		grdsects[i].degOfcurve = 360 * grdsects[i].arc / (2 * PI *grdsects[i].radius);

		//compensation for curve (temporary)
		double delgrade = 0.04 * grdsects[i].degOfcurve;
		if (delgrade > 0)
			grdsects[i].gradeWithcurve = grdsects[i].grade + delgrade;
		else
			grdsects[i].gradeWithcurve = grdsects[i].grade - delgrade;
	}
	
}

void RailwayExtraction::AddPointTofootOfPependicularOnRailVtx(vector<POINT3D> &Railvtx, POINT3D pt, int ptType)
{
	//Check if a point is on the rail vectors
	bool flg = false;
	for (int i = 0; i < Railvtx.size(); i++) {
		if (Railvtx[i].x == pt.x && Railvtx[i].y == pt.y) {
			flg = true;
			Railvtx[i].PtType = ptType;
			break;
		}
	}
	if (flg == true)
		return;

	////add vertex if foot of perpendicular of a point is between two rail points 
	flg = false;
	for (int i = 0; i < Railvtx.size() - 1; i++) {
		Line3D ln;
		ln.BeginPt = Railvtx[i];
		ln.EndPt = Railvtx[i + 1];
		ln.GetLineParameter();
		double lmaxX = -1e100, lminX = 1e100, lmaxY = -1e100, lminY = 1e100;

		lmaxX = max(ln.BeginPt.x, ln.EndPt.x);
		lminX = min(ln.BeginPt.x, ln.EndPt.x);
		lmaxY = max(ln.BeginPt.y, ln.EndPt.y);
		lminY = min(ln.BeginPt.y, ln.EndPt.y);

		POINT3D footPt = ComputeFootOfPerpendicular(pt, ln);
		if (footPt.x > lminX && footPt.x < lmaxX && footPt.y > lminY && footPt.y < lmaxY) {
			flg = true;
			footPt.PtType = ptType;
			Railvtx.insert(Railvtx.begin() + i + 1, footPt);
			Railvtx[i+1].PtType = ptType;
			break;
		}
	}

	//if not the case, the nearest point is considered as foot of perpendicular 
	if (flg == false) {
		double minDist = 1e100;
		int idx = -1;
		for (int i = 0; i < Railvtx.size(); i++) {
			double fdist = DistanceBetweenPoints2D(Railvtx[i], pt);
			if (fdist < minDist) {
				minDist = fdist;
				idx = i;
			}
		}
		if (idx != -1)
			Railvtx[idx].PtType = ptType;
	}

}

double RailwayExtraction::GetAngle(double x1, double y1, double x2, double y2) {
	double dX = x2 - x1;
	double dY = y2 - y1;
	double dXY = sqrt(dX*dX + dY*dY);
	double fAngle;

	if (dY >= 0) {
		if (dX >= 0)
			fAngle = acos(fabs(dX / dXY));
		else
			fAngle = PI - acos(fabs(dX / dXY));
	}
	else {
		if (dX<0)
			fAngle = acos(fabs(dX / dXY));
		else
			fAngle = PI - acos(fabs(dX / dXY));
	}

	if (bIsNan(fAngle))
		fAngle = 0;

	return  fAngle * 180 / PI;
}

void RailwayExtraction::ReserverVectors() {

	for (int i = 0; i < m_trackLayers.size(); i++) {

		TrackLayer* layer = m_trackLayers[i];
		std::reverse(layer->m_RailVec1.begin(), layer->m_RailVec1.end());
		std::reverse(layer->m_RailVec2.begin(), layer->m_RailVec2.end());
		std::reverse(layer->m_RailVecCenter.begin(), layer->m_RailVecCenter.end());

		layer->m_RailVec1.swap(layer->m_RailVec2);

	}

	currentLayer = m_trackLayers[0];

	currentLayer->m_RailVec1.pop_back();
	currentLayer->m_RailVec2.pop_back();
	currentLayer->m_RailVecCenter.pop_back();

}