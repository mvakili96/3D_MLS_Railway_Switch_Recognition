#pragma once
#include "LIDARPT.h"
#include <pcl/common/common_headers.h>
#include "PointXYZDouble.h"

#include "etc/matrix.h"
#include <vector>
#include "LASFileReader.h"
#include "TXTFileReader.h"

#include "VTKWindowController.h"
#include "LIDARPT.h"

#include "Viewer.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

using namespace System;
using namespace System::Windows::Forms;
using namespace System::Drawing;

class RailwayExtraction
{

private:
	vtkSmartPointer<vtkPoints> points;
	vtkSmartPointer<vtkCellArray> cell;
	vtkSmartPointer<vtkDoubleArray> depthArray;
	double minDepth, maxDepth;
	vtkSmartPointer<vtkDoubleArray> intensityArray;
	double minIntensity, maxIntensity;

public:
	RailwayExtraction(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud);
	~RailwayExtraction();
	void setInitialLocation();
	void generateGrid();

public:

	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;
	

	POINT3D m_BoundMax;
	POINT3D m_BoundMin;

	std::vector<TrackSegment> m_TrackSegList;

	bool m_bInit;
	double m_distFromSp;
	double m_InitTrkHgt1;
	double m_InitTrkHgt2;

	double m_GridSize;
	Grid* m_pGrid;
	double m_NumGrid;
	double m_NumX;
	double m_NumY;
	int m_skip_rate;

	double m_gLength;
	double m_gWidth;
	double m_ThRansac;
	double m_TrkGuage;
	double m_wMDL;
	double m_ThDistGrouping;
	int m_ThMinTrkPts;
	int m_ThMinPts;
	int m_ThMinIntensityForTrack;
	int m_ThMaxIntensityForTrack;

	std::vector<SwitchBox> m_SwitchBoxes;
	std::vector<LoopCross> m_LoopCross;
	std::vector<ToePt> m_ToePts;
	std::vector<TrackStack> m_TrkStks;


	std::vector <TrackLayer *> m_trackLayers;
	TrackLayer *currentLayer;

	//Template Matching
	std::vector<BinaryString> m_TmpltBegin;
	std::vector<BinaryString> m_TmpltEnd;
	std::vector<BinaryString> m_Tmplts;

	//visualization
//	std::vector<POINT3D> m_RailVec1;
//	std::vector<POINT3D> m_RailVec2;
//	std::vector<POINT3D> m_RailVecCenter;
//	std::vector<Line3D> m_updateBnd;
//	std::vector<Line3D> m_updateBnd_turnout;
//	std::vector<Line3D> m_updateBnd_loop;
//	std::vector<POINT3D> m_PredPts;
//	std::vector<LoopCross> m_visulLoopCross;

	VTKWindowController *windowController;
	UnmanagedProgressCallback progressCallback;

	int defaultColor_R = 125;
	int defaultColor_G = 125;
	int defaultColor_B = 125;


	void updateMaxMin();
	void updateMaxMin(double *maxPoint, double *minPoint);
	void updateMaxMin(PointXYZDouble maxPoint, PointXYZDouble minPoint);
	void updateMaxMinFromReader(LASFileReader *reader, const char *filename);
	void readMaxMinFromReader(LASFileReader *reader, const char *filename);
	void readMaxMinFromTXTReader(TXTFileReader *txtReader);

	void updateView(int m_ClassView);

	void OnRailextractionSetinit();
	void OnRailextractionSetinit(int mode, POINT3D pt, float angle);
	void OnRailextractionTrackingRailway();

	void ComputeGMMfeature(vector<PointXYZDouble*> points, vector<GMMPara> &paras, int _nClass);
	POINT3D ComputeFootOfPerpendicular(POINT3D p, Line3D ln);
	double ComputeInnerAngle(POINT3D sp, POINT3D mp, POINT3D ep);
	POINT3D ComputeIntersectionPoint3D(double A1, double B1, double C1, double A2, double B2, double C2);
	void ClusteringByGMM(vector<PointXYZDouble*> pnts, vector<GMMPara> paras, vector<vector<PointXYZDouble*>> &cPnts);
	void ExtractPlaneByRANSAC(vector<PointXYZDouble*> inPts, PlnPara &para, double Thresh, vector<PointXYZDouble*> &sptPnts);
	bool LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line, double &sigma, vector<PointXYZDouble*> &sptPnts);
	bool LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line);
	bool LineExtractionWithFixedAngle(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_ln, vector<PointXYZDouble*> &sptPnts);
	bool LineExtractionWithFixedAngleForLoopCross(vector<PointXYZDouble*> pnts, double ang, double ThDist, Line3D &_line, vector<PointXYZDouble*> &sptPnts);
	double DistanceBetweenPoints2D(POINT3D p1, POINT3D p2);
	double DistanceBetweenPoints3D(POINT3D p1, POINT3D p2);
	double DistanceBetweenPoints2D(PointXYZDouble* p1, PointXYZDouble* p2);
	double DistanceBetweenPoints3D(PointXYZDouble* p1, PointXYZDouble* p2);
	double DistanceBetweenPointAndLine2D(POINT3D p, Line3D l);
	double DistanceBetweenPointAndLine2D(PointXYZDouble* p, Line3D l);
	double DistanceBetweenPointAndLine3D(POINT3D _p, Line3D _ls);
	double DistanceBetweenPointAndLine3DHgt(PointXYZDouble* _p, Line3D _ls);
	double DistanceBetweenPointAndPlane(POINT3D p, PlnPara para);
	double ComputeArea(vector<POINT3D> vtx);
	int ReCount(int totnum, int curnum);
	vector<int> GetNeighborIdx(double _x, double _y, int _NumX, int _NumY, double _gSize, int _nSize);
	int GetDirectionMode(double nx, double ny, double px, double py);
	void GetPntsInsidePolygon(vector<POINT3D> inVtx, vector<PointXYZDouble*> inPts, vector<PointXYZDouble*> &outPts);
	void GetLnsFromVtx(vector<POINT3D> pts, vector<Line3D> &Lns);
	void GetMinMaxXY(vector<PointXYZDouble*> pnts, double &minX, double &maxX, double &minY, double &maxY);

	void KalmanPredict(int mode, double ang, double delL, Matrix<double> curPt, Matrix<double> curP, Matrix<double> &nextPt, Matrix<double> &nextP);
	void KalmanUpdate(Matrix<double> predPt, Matrix<double> predP, Matrix<double> obsPt, double sigma, Matrix<double> &hatPt, Matrix<double> &hatP);
	void SegmentPoints(vector<PointXYZDouble*> Pnts, vector<PointXYZDouble*> &segPnts, double fDistTh);
	void PointGrowing(PointXYZDouble* pCurVh, vector<PointXYZDouble*> Pnts, PtSegment &PtSegments, int SegID, double fDistTh);

	void DetectSwitchBox(vector<ToePt> toes, vector<SwitchBox> &SBs);
	void DetectLoopCross(TrackSegment Seg, vector<PointXYZDouble*> Pts, double HgtOfGround, double HgtOfTrack, Line3D refLn, vector<LoopCross> &LC);
	bool DetectToePnts(int type, vector<TrackSegment> trkSeg, ToePt &toe);
	void TemplateMatching(vector<BinaryString> tmplates, BinaryString &trkbinary);
	void RecognitionTurnoutType(vector<BinaryString> tmplates, BinaryString preBinary, BinaryString &trkbinary);
	void ValidateLoopCross(vector<LoopCross> &loopcross);
	void DefineTemplates();

	void Trackingrailway(int TrackDirection, bool mainTrkflg);
	bool Setinitiallocation(int inMode, POINT3D inPt, double inAng);
	void DetermineObservation(int TrackMode, TrackStack trkStk, Line3D guideln1, Line3D guideln2, vector<Line3D> lnList1, vector<Line3D> lnList2, int lnMode, vector<PointXYZDouble*> &obsPts1, vector<PointXYZDouble*> &obsPts2, vector<PointXYZDouble*> &LeftPts1, vector<PointXYZDouble*> &RightPts1, vector<PointXYZDouble*> &LeftPts2, vector<PointXYZDouble*> &RightPts2);
	void SegmentTrackPtsByMDL(int mode, vector<PointXYZDouble*> Pts, vector<Line3D> &lnList, double ang);
	double ComputeCloseness(vector<PointXYZDouble*> Pnts, vector<Line3D> lns);
	void AdjustTwoLines(Line3D &ln1, Line3D &ln2, int m1, int m2, int m, double _dist);
	void AdjustTwoLinesWithObservation(POINT3D ref1, POINT3D ref2, Line3D &ln1, Line3D &ln2, vector<PointXYZDouble*> sptPts1, vector<PointXYZDouble*> sptPts2, int m1, int m2, int m, double _dist);


	void GenerateTrkSegAssignMemPts(TrackSegment &nextSeg, TrackSegment curSeg, Matrix<double> predPt1, Matrix<double> predPt2, double Length, double Width, int mode);
	void DeleteSegPntsCmpWithPreHgt(TrackSegment &nextSeg, double aveHgtOfTrack1, double aveHgtOfTrack2, double aveHgtOfGround1, double aveHgtOfGround2, double ThHgtOfTrack, double ThHgtOfGround);
	void ComputeGMMParaAndCluster(TrackSegment nextSeg, vector<GMMPara> &nextParas1, vector<GMMPara> &nextParas2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2);
	void ExtractTrkPts(TrackSegment nSeg, double curTrkHgt1, double curTrkHgt2, vector<PointXYZDouble*> &HeadPts1, vector<PointXYZDouble*> &HeadPts2, double &nextTrkHgt1, double &nextTrkHgt2);

	vector<GradeSect> ComputeGrade(TrackLayer *layer);
	void DetermineGradeSection(vector<POINT3D> &railVtx, double thDPv);
	void DouglasPeuckerVertical(vector<POINT3D> &railVtx, int _sIdx, int _eIdx, double thDP);
	void ComputeGradeParameters(vector<POINT3D> railVtx, vector<GradeSect> &grdsects);
	void AddPointTofootOfPependicularOnRailVtx(vector<POINT3D> &Railvtx, POINT3D pt, int ptType);

	void PrintTurnoutType(int type);
	void PrintVtx(vector<POINT3D> vtx);
	void PrintPts(vector<PointXYZDouble*> pts);
	void PrintLines(vector<Line3D> lns);

	PlnPara GetPlaneParameters3p(POINT3D p1, POINT3D p2, POINT3D p3);
	PlnPara GetPlaneParametersEigen(vector<PointXYZDouble*> pts);

	InitialLocation initialLocation;

	std::vector<TrackLayer *> GetLayers();

	void updateView(int m_ClassView, TreeNodeCollection ^collection);

	void updateExtractionParameters(ExtractionParameters parameters);
	ExtractionParameters getExtractionParameters();

	void ReserverVectors();

	double GetAngle(double x1, double y1, double x2, double y2);
};

