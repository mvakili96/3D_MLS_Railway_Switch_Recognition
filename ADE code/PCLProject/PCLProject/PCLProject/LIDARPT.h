#ifndef _LIDARPT_H_
#define  _LIDARPT_H_

//#include "etc/matrix.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include "Jama/jama_eig.h"

#define MAX_LINE_LEN 2048
#define DEFAULT_TOOCLOSE 0.25

#ifndef PI 
#define PI	3.1415926535897932
#endif

#define T_OFF 0
#define T_BUFF1 1
#define T_BUFF2 2
#define T_ON 3

#include "PointXYZDouble.h"

using namespace std;

inline bool bIsNan(double d)
{ 
	return (d)!=(d); 
}

inline int FLOOR_double (float _f) {
	float fDec = _f-int(_f);

	if(fDec>=0.5)
		return int(_f)+1;
	else
		return int(_f);
}

/*inline void Rotation3D(double &_X, double &_Y, double &_Z, double _Ax, double _Ay, double _Az, double _Tx, double _Ty, double _Tz)
{
	Matrix<double> mOriginXYZ(3,1,0);
	mOriginXYZ(0,0) = _X - _Tx;
	mOriginXYZ(1,0) = _Y - _Ty;
	mOriginXYZ(2,0) = _Z - _Tz;

	Matrix<double> mRotX(3,3,0), mRotY(3,3,0), mRotZ(3,3,0);
	mRotX(0,0) = 1;
	mRotX(1,1) = cos(_Ax);
	mRotX(1,2) = sin(_Ax);
	mRotX(2,1) = -sin(_Ax);
	mRotX(2,2) = cos(_Ax);

	mRotY(0,0) = cos(_Ay);
	mRotY(0,2) = -sin(_Ay);
	mRotY(1,1) = 1;
	mRotY(2,0) = sin(_Ay);
	mRotY(2,2) = cos(_Ay);

	mRotZ(0,0) = cos(_Az);
	mRotZ(0,1) = sin(_Az);
	mRotZ(1,0) = -sin(_Az);
	mRotZ(1,1) = cos(_Az);
	mRotZ(2,2) = 1;

	Matrix<double> mRotation = mRotX*mRotY*mRotZ;
	//Matrix<double> mRotation = mRotZ*mRotY*mRotX;
	Matrix<double> mRotXYZ = mRotation*mOriginXYZ;;

	_X = mRotXYZ(0,0);
	_Y = mRotXYZ(1,0);
	_Z = mRotXYZ(2,0);
};

inline void InverseRotation3D(double &_X, double &_Y, double &_Z, double _Ax, double _Ay, double _Az, double _Tx, double _Ty, double _Tz)
{
	Matrix<double> mOriginXYZ(3,1,0);
	mOriginXYZ(0,0) = _X;
	mOriginXYZ(1,0) = _Y;
	mOriginXYZ(2,0) = _Z;

	Matrix<double> mRotX(3,3,0), mRotY(3,3,0), mRotZ(3,3,0);
	mRotX(0,0) = 1;
	mRotX(1,1) = cos(_Ax);
	mRotX(1,2) = sin(_Ax);
	mRotX(2,1) = -sin(_Ax);
	mRotX(2,2) = cos(_Ax);

	mRotY(0,0) = cos(_Ay);
	mRotY(0,2) = -sin(_Ay);
	mRotY(1,1) = 1;
	mRotY(2,0) = sin(_Ay);
	mRotY(2,2) = cos(_Ay);

	mRotZ(0,0) = cos(_Az);
	mRotZ(0,1) = sin(_Az);
	mRotZ(1,0) = -sin(_Az);
	mRotZ(1,1) = cos(_Az);
	mRotZ(2,2) = 1;

	Matrix<double> mRotation = mRotX*mRotY*mRotZ;
	Matrix<double> mRotXYZ= mRotation.Transpose()*mOriginXYZ;;

	_X = mRotXYZ(0,0) + _Tx;
	_Y = mRotXYZ(1,0) + _Ty;
	_Z = mRotXYZ(2,0) + _Tz;
};*/

inline int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
	int i, j, c = 0;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((verty[i]>testy) != (verty[j]>testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
			c = !c;
	}
	return c;
};

inline int pnpoly2(int nvert, double *vertx, double *verty, double testx, double testy)
{
	double fPrecision = 1e-10;
	vector<int> vTopos;
	for (int i = 0; i<nvert; i++) {
		double pt1x = vertx[(i + 1) % nvert] - vertx[i];
		double pt1y = verty[(i + 1) % nvert] - verty[i];
		double pt2x = testx - vertx[i];
		double pt2y = testy - verty[i];

		// cross product
		double fZElem = pt1x*pt2y - pt1y*pt2x;

		int nTopo;
		if (fabs(fZElem)<fPrecision)
			nTopo = 0;
		else {
			if (fZElem < 0)
				nTopo = -1;
			else
				nTopo = 1;
		}
		vTopos.push_back(nTopo);
	}

	sort(vTopos.begin(), vTopos.end());
	if (vTopos.front() == vTopos.back())
		return 1;
	else
		return 0;
};

class LiDPOINT
{
public:
	//	union
	//	{
	//		struct {REAL x,y,z;};
	//		REAL _m[3];
	//	};
	double x, y, z;

	// Las Format
	int		SegmentID;
	int		ClassCode;		//default: 0, rail(above point): 1, non-rail (below point): 2
	unsigned short		Intensity;
	//	USHORT	NumOfReturns;
	//	USHORT	ReturnNumber;
	// Las Format - END
	//	USHORT AngleIndex;		//0:0-22.5, 1:22.5-45, 2:45-67.5, 3:67.5-90, 4:90-112.5, 5:112.5-135, 6:135-157.5, 7:157.5-180

	//added temporary variables for visualization
	double dAlpha;
	double dHoG;
	double dZ;
	bool deltag;

	LiDPOINT() {
		Intensity = 0;
		//		NumOfReturns = 0;
		//		ReturnNumber = 0;
		//		AngleIndex = -1;
		ClassCode = 0;
		dAlpha = 0;
		dHoG = 0;
		dZ = 0;
		deltag = false;
		SegmentID = -1;
	};

	LiDPOINT(double _x, double _y, double _z) {
		Intensity = 0;
		//		NumOfReturns = 0;
		//		ReturnNumber = 0;
		//		AngleIndex = -1;
		ClassCode = 0;
		x = _x;
		y = _y;
		z = _z;
		dAlpha = 0;
		dHoG = 0;
		dZ = 0;
		deltag = false;
		SegmentID = -1;
	};

	~LiDPOINT() {}

	void SetXYZ(double _x, double _y, double _z) {
		x = _x;
		y = _y;
		z = _z;
	}

	void SetXYZ(LiDPOINT pt) {
		x = pt.x;
		y = pt.y;
		z = pt.z;
	}

	bool IsEqual(LiDPOINT _pt) {
		double fPrecision = 1e-5;
		if (fabs(x - _pt.x)<fPrecision && fabs(y - _pt.y)<fPrecision && fabs(z - _pt.z)<fPrecision)
			return true;
		else
			return false;
	}

	LiDPOINT operator- (const LiDPOINT _pt) {
		LiDPOINT Pt;
		Pt.x = x - _pt.x;
		Pt.y = y - _pt.y;
		Pt.z = z - _pt.z;

		return Pt;
	}

	LiDPOINT operator+ (const LiDPOINT _pt) {
		LiDPOINT Pt;
		Pt.x = x + _pt.x;
		Pt.y = y + _pt.y;
		Pt.z = z + _pt.z;

		return Pt;
	}

	LiDPOINT operator/ (const double _s) {
		LiDPOINT Pt;
		Pt.x /= _s;
		Pt.y /= _s;
		Pt.z /= _s;

		return Pt;
	}
};

struct POINT2D
{
	double x, y;
	bool flg;

	POINT2D() {
		x = 0; y = 0;
		flg = false;
	}

	POINT2D(double _x, double _y) {
		x = _x; y = _y;
	}

	bool IsEqual(POINT2D _pt) {
		double fPrecision = 1e-5;
		if (fabs(x - _pt.x)<fPrecision && fabs(y - _pt.y)<fPrecision)
			return true;
		else
			return false;
	}

	POINT2D operator* (const double _s) {
		POINT2D Pt;
		Pt.x = x * _s;
		Pt.y = y * _s;

		return Pt;
	}

	POINT2D operator/ (const double _s) {
		if (_s == 0)
			return *this;

		POINT2D Pt;
		Pt.x = x / _s;
		Pt.y = y / _s;

		return Pt;
	}

	POINT2D operator- (const POINT2D _pt) {
		POINT2D Pt;
		Pt.x = x - _pt.x;
		Pt.y = y - _pt.y;

		return Pt;
	}

	POINT2D operator+ (const POINT2D _pt) {
		POINT2D Pt;
		Pt.x = x + _pt.x;
		Pt.y = y + _pt.y;

		return Pt;
	}

	bool isEqual(POINT2D _pt) {
		if (x == _pt.x && y == _pt.y)
			return true;
		else
			return false;
	}

};

struct POINT3D {
public:
	union
	{
		struct { double x, y, z; };
		double _m[3];
	};

	int PtType;				//Point type 0:Rail track vector, 1:Grade poitn, 2:Switch box, ...
	//	double tmpValue;

	POINT3D() {
		PtType = 0;	
	};

	POINT3D(double _x, double _y, double _z) {
		x = _x;
		y = _y;
		z = _z;
		PtType = 0;
	}

	void SetPoint3D(double _x, double _y, double _z) {
		x = _x;
		y = _y;
		z = _z;
		PtType = 0;
	}

	void ChangeSign() {
		x = -x;
		y = -y;
		z = -z;
	}

	void Normalize() {
		double fR = sqrt(x*x + y*y + z*z);
		x /= fR;
		y /= fR;
		z /= fR;
	}

	void operator= (POINT3D _pt3d) {
		x = _pt3d.x;
		y = _pt3d.y;
		z = _pt3d.z;
	};

	POINT3D operator- (POINT3D _pt3d) {
		POINT3D pt3d;
		pt3d.x = x - _pt3d.x;
		pt3d.y = y - _pt3d.y;
		pt3d.z = z - _pt3d.z;

		return(pt3d);
	}

	POINT3D operator+ (POINT3D _pt3d) {
		POINT3D pt3d;
		pt3d.x = x + _pt3d.x;
		pt3d.y = y + _pt3d.y;
		pt3d.z = z + _pt3d.z;

		return(pt3d);
	}

	POINT3D operator* (double _s) {
		POINT3D pt3d;
		pt3d.x = x * _s;
		pt3d.y = y * _s;
		pt3d.z = z * _s;

		return(pt3d);
	}

	POINT3D operator/ (double _s) {
		POINT3D pt3d;
		pt3d.x = x / _s;
		pt3d.y = y / _s;
		pt3d.z = z / _s;

		return(pt3d);
	}

	bool IsEqual(POINT3D _pt) {
		double fPrecision = 1e-5;
		if (fabs(x - _pt.x)<fPrecision && fabs(y - _pt.y)<fPrecision && fabs(z - _pt.z)<fPrecision)
			return true;
		else
			return false;
	}

	double DotProduct(POINT3D _pt3d) {
		return (x*_pt3d.x + y*_pt3d.y + z*_pt3d.z);
	}

	POINT3D CrossProduct(POINT3D _pt3d) {
		return POINT3D(y*_pt3d.z - _pt3d.y*z, _pt3d.x*z - x*_pt3d.z, x*_pt3d.y - _pt3d.x*y);
	}


};

struct Line3D {
	POINT3D BeginPt;
	POINT3D EndPt;
	POINT3D CenterPt;
	double Ang;
	double A2D, B2D, C2D;		//line parameter Ax+By+C=0;
	POINT3D m_UnitVec;
	double D3D;
	//	double A3D,B3D,C3D,D3D;
	vector<PointXYZDouble*> MemPts;

	Line3D() {
	}

	void GetAngle() {
		double dX = EndPt.x - BeginPt.x;
		double dY = EndPt.y - BeginPt.y;
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

		Ang = fAngle * 180 / PI;
	}

	void GetLineParameter() {
		double dX = EndPt.x - BeginPt.x;
		double dY = EndPt.y - BeginPt.y;
		if (dX == 0) {
			A2D = 1;
			B2D = 0;
			C2D = -1 * BeginPt.x;
		}
		else if (dY == 0) {
			A2D = 0;
			B2D = -1;
			C2D = BeginPt.y;
		}
		else {
			A2D = dY / dX;
			B2D = -1;
			C2D = BeginPt.y - A2D*BeginPt.x;
		}
	}
	void Get2DLineParameterWithAngleAndPt(double _x, double _y)
	{
		if (Ang == 0) {
			A2D = 0;
			B2D = -1;
			C2D = _x;
		}
		else if (Ang == 90) {
			B2D = 0;
			A2D = -1;
			C2D = _y;
		}
		else {
			A2D = tan(Ang * PI / 180);
			B2D = -1;
			C2D = _y - A2D*_x;
		}
	}

	void GetCenterPt()
	{
		CenterPt.x = (BeginPt.x + EndPt.x) / 2;
		CenterPt.y = (BeginPt.y + EndPt.y) / 2;
		CenterPt.z = (BeginPt.z + EndPt.z) / 2;
	}

	void SwapEnding() {
		POINT3D tmpPt = BeginPt;
		BeginPt = EndPt;
		EndPt = tmpPt;
	}
	void SetEndingByMode(int mode) {
		double diffx = fabs(BeginPt.x - EndPt.x);
		double diffy = fabs(BeginPt.y - EndPt.y);

		if (diffx > diffy) {
			if (mode == 1) {
				if (BeginPt.x > EndPt.x)
					SwapEnding();
			}
			else if (mode == 2) {
				if (BeginPt.x < EndPt.x)
					SwapEnding();
			}
			else if (mode == 3) {
				if (BeginPt.x < EndPt.x)
					SwapEnding();
			}
			else {
				if (BeginPt.x > EndPt.x)
					SwapEnding();
			}
		}
		else {
			if (mode == 1) {
				if (BeginPt.y > EndPt.y)
					SwapEnding();
			}
			else if (mode == 2) {
				if (BeginPt.y > EndPt.y)
					SwapEnding();
			}
			else if (mode == 3) {
				if (BeginPt.y < EndPt.y)
					SwapEnding();
			}
			else {
				if (BeginPt.y < EndPt.y)
					SwapEnding();
			}
		}
	}
	void SetEndByReferencePt(POINT3D ref) {
		//Closer point is beginPt
		double fdist1 = sqrt((BeginPt.x - ref.x)*(BeginPt.x - ref.x) + (BeginPt.y - ref.y)*(BeginPt.y - ref.y));
		double fdist2 = sqrt((EndPt.x - ref.x)*(EndPt.x - ref.x) + (EndPt.y - ref.y)*(EndPt.y - ref.y));

		if (fdist1 > fdist2)
			SwapEnding();

	}
	Line3D ExtendEndingPts(double times) {
		Line3D line;
		GetCenterPt();
		POINT3D sp = CenterPt - BeginPt;
		sp = sp * times;
		line.BeginPt = CenterPt - sp;
		POINT3D ep = CenterPt - EndPt;
		ep = ep * times;
		line.EndPt = CenterPt - ep;
		return line;
	}

	bool IsVertex(POINT3D _pt) {
		if (BeginPt.IsEqual(_pt))
			return true;
		else if (EndPt.IsEqual(_pt)) {
			SwapVertex();
			return true;
		}
		else
			return false;
	}

	void SwapVertex() {
		POINT3D temp;
		temp = BeginPt;
		BeginPt = EndPt;
		EndPt = temp;
	}

	int GetTopologyUsingCrossProduct(double x, double y) {
		POINT3D curPt(x, y, 0);
		POINT3D pt1 = curPt - EndPt;
		POINT3D pt2 = EndPt - BeginPt;

		// cross product
		double fZElem = pt1.x*pt2.y - pt1.y*pt2.x;

		if (fZElem > 0)
			return 1;
		else if (fZElem < 0)
			return -1;
		else
			return 0;
	}

	void SetEndingPnts(double minX, double maxX, double minY, double maxY) {
		if (A2D == 0 && B2D == 0)
			return;
		else if (A2D == 0) {
			BeginPt.x = minX;
			BeginPt.y = -C2D / B2D;
			//	BeginPt.z = 0;
			EndPt.x = maxX;
			EndPt.y = -C2D / B2D;
			//	EndPt.z = 0;
		}
		else if (B2D == 0) {
			BeginPt.x = -C2D / A2D;
			BeginPt.y = minY;
			//	BeginPt.z = 0;
			EndPt.x = -C2D / A2D;
			EndPt.y = maxY;
			//	EndPt.z = 0;
		}
		else if (-A2D / B2D<1) {
			BeginPt.x = minX;
			BeginPt.y = -A2D / B2D*BeginPt.x - C2D / B2D;
			//	BeginPt.z = 0;
			EndPt.x = maxX;
			EndPt.y = -A2D / B2D*EndPt.x - C2D / B2D;
			//	EndPt.z = 0;
		}
		else {
			BeginPt.y = minY;
			BeginPt.x = -B2D / A2D*BeginPt.y - C2D / A2D;
			//	BeginPt.z = 0;
			EndPt.y = maxY;
			EndPt.x = -B2D / A2D*EndPt.y - C2D / A2D;
			//	EndPt.z = 0;
		}
	}

	void SetEndingPnts3D(double minX, double maxX, double minY, double maxY) {
		if (A2D == 0 && B2D == 0)
			return;
		else if (A2D == 0) {
			BeginPt.x = minX;
			BeginPt.y = -C2D / B2D;
			BeginPt.z = 0;
			//		BeginPt.z = -(m_UnitVec.x*BeginPt.x+m_UnitVec.y*BeginPt.y+D3D)/m_UnitVec.z;
			EndPt.x = maxX;
			EndPt.y = -C2D / B2D;
			EndPt.z = 0;
			//		EndPt.z = -(m_UnitVec.x*EndPt.x+m_UnitVec.y*EndPt.y+D3D)/m_UnitVec.z;
		}
		else if (B2D == 0) {
			BeginPt.x = -C2D / A2D;
			BeginPt.y = minY;
			BeginPt.z = 0;
			//		BeginPt.z = -(m_UnitVec.x*BeginPt.x+m_UnitVec.y*BeginPt.y+D3D)/m_UnitVec.z;
			EndPt.x = -C2D / A2D;
			EndPt.y = maxY;
			EndPt.z = 0;
			//		EndPt.z = -(m_UnitVec.x*EndPt.x+m_UnitVec.y*EndPt.y+D3D)/m_UnitVec.z;
		}
		else if (-A2D / B2D<1) {
			BeginPt.x = minX;
			BeginPt.y = -A2D / B2D*BeginPt.x - C2D / B2D;
			BeginPt.z = 0;
			//		BeginPt.z = -(m_UnitVec.x*BeginPt.x+m_UnitVec.y*BeginPt.y+D3D)/m_UnitVec.z;
			EndPt.x = maxX;
			EndPt.y = -A2D / B2D*EndPt.x - C2D / B2D;
			EndPt.z = 0;
			//		EndPt.z = -(m_UnitVec.x*EndPt.x+m_UnitVec.y*EndPt.y+D3D)/m_UnitVec.z;
		}
		else {
			BeginPt.y = minY;
			BeginPt.x = -B2D / A2D*BeginPt.y - C2D / A2D;
			BeginPt.z = 0;
			//		BeginPt.z = -(m_UnitVec.x*BeginPt.x+m_UnitVec.y*BeginPt.y+D3D)/m_UnitVec.z;
			EndPt.y = maxY;
			EndPt.x = -B2D / A2D*EndPt.y - C2D / A2D;
			EndPt.z = 0;
			//		EndPt.z = -(m_UnitVec.x*EndPt.x+m_UnitVec.y*EndPt.y+D3D)/m_UnitVec.z;
		}
	}

	void SetEndingPntsByCenterAndLength(double radius) {
		double tmpAng = Ang * PI / 180;
		BeginPt.x = CenterPt.x + radius*cos(tmpAng);
		BeginPt.y = CenterPt.y + radius*sin(tmpAng);
		BeginPt.z = CenterPt.z;
		EndPt.x = CenterPt.x - radius*cos(tmpAng);
		EndPt.y = CenterPt.y - radius*sin(tmpAng);
		EndPt.z = CenterPt.z;
	}

	void GetUnitVec() {
		double Dx = EndPt.x - BeginPt.x;
		double Dy = EndPt.y - BeginPt.y;
		double Dz = EndPt.z - BeginPt.z;

		double fdist = sqrt(Dx*Dx + Dy*Dy + Dz*Dz);

		m_UnitVec.x = Dx / fdist;
		m_UnitVec.y = Dy / fdist;
		m_UnitVec.z = Dz / fdist;
	}

	vector<POINT3D> GetBufferVtxOfLine(double buffersize) {
		vector<POINT3D> plist;
		if (A2D != 0 && B2D != 0) {
			POINT3D pnt(0, 0, 0);
			pnt.x = 0;
			pnt.y = -1 * C2D / B2D;

			Line3D pline1, pline2;
			pline1.A2D = A2D;
			pline1.B2D = B2D;
			pline1.C2D = buffersize*sqrt(pline1.A2D* pline1.A2D + pline1.B2D*pline1.B2D) - pline1.A2D * pnt.x - pline1.B2D * pnt.y;

			pline2.A2D = A2D;
			pline2.B2D = B2D;
			pline2.C2D = -1 * buffersize*sqrt(pline2.A2D* pline2.A2D + pline2.B2D*pline2.B2D) - pline2.A2D * pnt.x - pline2.B2D * pnt.y;

			Line3D oline1, oline2;
			oline1.A2D = 1 / A2D;
			oline1.B2D = -1 * B2D;
			oline1.C2D = -1 * (oline1.A2D * BeginPt.x + oline1.B2D * BeginPt.y);

			oline2.A2D = 1 / A2D;
			oline2.B2D = -1 * B2D;
			oline2.C2D = -1 * (oline2.A2D * EndPt.x + oline2.B2D * EndPt.y);

			pnt.x = -1 * (pline1.C2D + oline1.C2D) / (pline1.A2D + oline1.A2D);
			pnt.y = -1 * (pline1.A2D * pnt.x + pline1.C2D) / pline1.B2D;

			plist.push_back(pnt);

			pnt.x = -1 * (pline1.C2D + oline2.C2D) / (pline1.A2D + oline2.A2D);
			pnt.y = -1 * (pline1.A2D * pnt.x + pline1.C2D) / pline1.B2D;

			plist.push_back(pnt);

			pnt.x = -1 * (pline2.C2D + oline2.C2D) / (pline2.A2D + oline2.A2D);
			pnt.y = -1 * (pline2.A2D * pnt.x + pline2.C2D) / pline2.B2D;

			plist.push_back(pnt);

			pnt.x = -1 * (pline2.C2D + oline1.C2D) / (pline2.A2D + oline1.A2D);
			pnt.y = -1 * (pline2.A2D * pnt.x + pline2.C2D) / pline2.B2D;

			plist.push_back(pnt);
		}
		else if (A2D == 0) {
			POINT3D pnt(0, 0, 0);
			pnt.x = BeginPt.x; pnt.y = C2D - buffersize;
			plist.push_back(pnt);
			pnt.x = EndPt.x; pnt.y = C2D - buffersize;
			plist.push_back(pnt);
			pnt.x = EndPt.x; pnt.y = C2D + buffersize;
			plist.push_back(pnt);
			pnt.x = BeginPt.x; pnt.y = C2D + buffersize;
			plist.push_back(pnt);
		}
		else if (B2D == 0) {
			POINT3D pnt(0, 0, 0);
			pnt.x = -1 * C2D - buffersize; pnt.y = BeginPt.y;
			plist.push_back(pnt);
			pnt.x = -1 * C2D - buffersize; pnt.y = EndPt.y;
			plist.push_back(pnt);
			pnt.x = -1 * C2D + buffersize; pnt.y = EndPt.y;
			plist.push_back(pnt);
			pnt.x = -1 * C2D + buffersize; pnt.y = BeginPt.y;
			plist.push_back(pnt);
		}

		return plist;
	}
};

struct GMMPara {
	double mu;
	double sigma;
	double alpha;

	GMMPara() {
		mu = 0;
		sigma = 0;
		alpha = 0;
	}

	void operator= (const GMMPara para) {
		mu = para.mu;
		sigma = para.sigma;
		alpha = para.alpha;
	}
};

struct Grid {
	unsigned int m_nX;
	unsigned int m_nY;
	double minX, maxX, minY, maxY;

	vector<PointXYZDouble*> MemPnts;
};

struct TrackWindow {
	double ori;
	POINT3D CenterPt;
	vector<POINT3D> vtx;
	vector<POINT3D> slVtx;
	vector<POINT3D> srVtx;
	vector<POINT3D> smVtx;
	vector<PointXYZDouble*> MemPnts;
	vector<PointXYZDouble*> slMemPnts;
	vector<PointXYZDouble*> srMemPnts;
	Line3D MemLn;
	bool bTurnOut;
	int bLeftTag;
	int bRightTag;

	TrackWindow() {
		vtx.clear();
		MemPnts.clear();
		bTurnOut = false;
		bLeftTag = 0;		//0: no, 1: yes
		bRightTag = 0;
	}
	void ResetTrackWindow() {
		vtx.clear();
		MemPnts.clear();
		bTurnOut = false;
		bLeftTag = 0;
		bRightTag = 0;
	}

	void GenerateVtxUsingCenterAndAngle(double length, double width, int mode) {
		vtx.clear();
		POINT3D cp = CenterPt;
		double ang = ori * PI / 180;
		Line3D Ln;
		if (mode == 1) {
			Ln.BeginPt.x = CenterPt.x - fabs(length / 2 * cos(ang));
			Ln.BeginPt.y = CenterPt.y - fabs(length / 2 * sin(ang));
			Ln.BeginPt.z = CenterPt.z;
			Ln.EndPt.x = CenterPt.x + fabs(length / 2 * cos(ang));
			Ln.EndPt.y = CenterPt.y + fabs(length / 2 * sin(ang));
			Ln.EndPt.z = CenterPt.z;
		}
		else if (mode == 2) {
			Ln.BeginPt.x = CenterPt.x + fabs(length / 2 * cos(ang));
			Ln.BeginPt.y = CenterPt.y - fabs(length / 2 * sin(ang));
			Ln.BeginPt.z = CenterPt.z;
			Ln.EndPt.x = CenterPt.x - fabs(length / 2 * cos(ang));
			Ln.EndPt.y = CenterPt.y + fabs(length / 2 * sin(ang));
			Ln.EndPt.z = CenterPt.z;
		}
		else if (mode == 3) {
			Ln.BeginPt.x = CenterPt.x + fabs(length / 2 * cos(ang));
			Ln.BeginPt.y = CenterPt.y + fabs(length / 2 * sin(ang));
			Ln.BeginPt.z = CenterPt.z;
			Ln.EndPt.x = CenterPt.x - fabs(length / 2 * cos(ang));
			Ln.EndPt.y = CenterPt.y - fabs(length / 2 * sin(ang));
			Ln.EndPt.z = CenterPt.z;
		}
		else {
			Ln.BeginPt.x = CenterPt.x - fabs(length / 2 * cos(ang));
			Ln.BeginPt.y = CenterPt.y + fabs(length / 2 * sin(ang));
			Ln.BeginPt.z = CenterPt.z;
			Ln.EndPt.x = CenterPt.x + fabs(length / 2 * cos(ang));
			Ln.EndPt.y = CenterPt.y - fabs(length / 2 * sin(ang));
			Ln.EndPt.z = CenterPt.z;
		}

		Ln.GetLineParameter();
		Ln.GetAngle();
		vtx = Ln.GetBufferVtxOfLine(width / 2);

	}
	void SetTrackWindowMemPnts(vector<PointXYZDouble*> Pnts) {
		MemPnts.clear();
		vector<double> vX, vY;
		for (int i = 0; i<vtx.size(); i++) {
			vX.push_back(vtx[i].x);
			vY.push_back(vtx[i].y);
		}
		for (int i = 0; i<Pnts.size(); i++) {
			if (pnpoly(vX.size(), &(vX.front()), &(vY.front()), Pnts[i]->x, Pnts[i]->y) == true) {
				MemPnts.push_back(Pnts[i]);
			}
		}
	}
	void GetMaxMinOfMemPnts(POINT3D &maxPt, POINT3D &minPt) {
		POINT3D MinPt(1e100, 1e100, 1e100);
		POINT3D MaxPt(-1e100, -1e100, -1e100);
		for (int i = 0; i<MemPnts.size(); i++) {
			MinPt.x = min(MemPnts[i]->x, MinPt.x);
			MinPt.y = min(MemPnts[i]->y, MinPt.y);
			MinPt.z = min(MemPnts[i]->z, MinPt.z);
			MaxPt.x = max(MemPnts[i]->x, MaxPt.x);
			MaxPt.y = max(MemPnts[i]->y, MaxPt.y);
			MaxPt.z = max(MemPnts[i]->z, MaxPt.z);
		}
		maxPt = MaxPt;
		minPt = MinPt;
	}
	void GetSideVtx(Line3D ln, double centerWidth, double sideWidth) {
		slVtx.clear();
		srVtx.clear();
		smVtx.clear();

		Line3D tmpLn;
		vector<Line3D> lns;
		for (int i = 0; i<3; i++)
			lns.push_back(tmpLn);

		lns[1].BeginPt = ln.BeginPt; lns[1].EndPt = ln.EndPt; lns[1].GetLineParameter();

		POINT3D Vec(0, 0, 0);
		Vec.x = ln.A2D;
		Vec.y = ln.B2D;
		Vec = Vec / sqrt(Vec.x*Vec.x + Vec.y*Vec.y + Vec.z*Vec.z);

		lns[0].BeginPt = lns[1].BeginPt - Vec*(centerWidth / 2 + sideWidth / 2);
		lns[0].EndPt = lns[1].EndPt - Vec*(centerWidth / 2 + sideWidth / 2);
		lns[0].GetLineParameter();

		lns[2].BeginPt = lns[1].BeginPt + Vec*(centerWidth / 2 + sideWidth / 2);
		lns[2].EndPt = lns[1].EndPt + Vec*(centerWidth / 2 + sideWidth / 2);
		lns[2].GetLineParameter();

		slVtx = lns[0].GetBufferVtxOfLine(sideWidth / 2);
		smVtx = lns[1].GetBufferVtxOfLine(sideWidth / 2);
		srVtx = lns[2].GetBufferVtxOfLine(sideWidth / 2);
	}
};

struct TrackSegment {
	TrackWindow m_Track1;
	TrackWindow m_Track2;
	double m_D;
	double distFromSp;
	POINT3D m_Center;
	vector<POINT3D> mmVtxBND;
	vector<POINT3D> mlVtxBND;
	vector<POINT3D> mrVtxBND;
	vector<POINT3D> lVtxBND;
	vector<POINT3D> rVtxBND;

	void SetTrkGauge(double dist) {
		m_D = dist;
	}

	void GetNeighborVtxBND(double length, double width, double widthTrkWnd) {
		mmVtxBND.clear();
		mlVtxBND.clear();
		mrVtxBND.clear();
		lVtxBND.clear();
		rVtxBND.clear();

		POINT3D sp, ep, cp;
		Line3D mmLn, mlLn, mrLn, lLn, rLn;
		sp = m_Track1.CenterPt;
		ep = m_Track2.CenterPt;
		cp = (sp + ep) / 2;

		//for mmVtx
		double sLength = sqrt((sp.x - cp.x)*(sp.x - cp.x) + (sp.y - cp.y)*(sp.y - cp.y));
		mmLn.BeginPt.x = cp.x + (sLength - width)*(sp.x - cp.x) / sLength;
		mmLn.BeginPt.y = cp.y + (sLength - width)*(sp.y - cp.y) / sLength;
		double eLength = sqrt((ep.x - cp.x)*(ep.x - cp.x) + (ep.y - cp.y)*(ep.y - cp.y));
		mmLn.EndPt.x = cp.x + (eLength - width)*(ep.x - cp.x) / eLength;
		mmLn.EndPt.y = cp.y + (eLength - width)*(ep.y - cp.y) / eLength;
		mmLn.GetLineParameter();
		mmVtxBND = mmLn.GetBufferVtxOfLine(length / 2);

		//for mlVtx
		sLength = sqrt((sp.x - cp.x)*(sp.x - cp.x) + (sp.y - cp.y)*(sp.y - cp.y));
		mrLn.BeginPt.x = cp.x + (sLength - width)*(sp.x - cp.x) / sLength;
		mrLn.BeginPt.y = cp.y + (sLength - width)*(sp.y - cp.y) / sLength;
		mrLn.EndPt.x = cp.x + (sLength - widthTrkWnd / 2)*(sp.x - cp.x) / sLength;
		mrLn.EndPt.y = cp.y + (sLength - widthTrkWnd / 2)*(sp.y - cp.y) / sLength;
		mrLn.GetLineParameter();
		mrVtxBND = mrLn.GetBufferVtxOfLine(length / 2);

		//for mrVtx
		eLength = sqrt((ep.x - cp.x)*(ep.x - cp.x) + (ep.y - cp.y)*(ep.y - cp.y));
		mlLn.BeginPt.x = cp.x + (eLength - width)*(ep.x - cp.x) / eLength;
		mlLn.BeginPt.y = cp.y + (eLength - width)*(ep.y - cp.y) / eLength;
		mlLn.EndPt.x = cp.x + (eLength - widthTrkWnd / 2)*(ep.x - cp.x) / eLength;
		mlLn.EndPt.y = cp.y + (eLength - widthTrkWnd / 2)*(ep.y - cp.y) / eLength;
		mlLn.GetLineParameter();
		mlVtxBND = mlLn.GetBufferVtxOfLine(length / 2);

		//for rVtx
		sLength = sqrt((sp.x - cp.x)*(sp.x - cp.x) + (sp.y - cp.y)*(sp.y - cp.y));
		rLn.BeginPt.x = cp.x + (sLength + widthTrkWnd / 2)*(sp.x - cp.x) / sLength;
		rLn.BeginPt.y = cp.y + (sLength + widthTrkWnd / 2)*(sp.y - cp.y) / sLength;
		rLn.EndPt.x = cp.x + (sLength + 4 * widthTrkWnd / 2)*(sp.x - cp.x) / sLength;
		rLn.EndPt.y = cp.y + (sLength + 4 * widthTrkWnd / 2)*(sp.y - cp.y) / sLength;
		rLn.GetLineParameter();
		rVtxBND = rLn.GetBufferVtxOfLine(length / 2);

		//for lVtx
		eLength = sqrt((ep.x - cp.x)*(ep.x - cp.x) + (ep.y - cp.y)*(ep.y - cp.y));
		lLn.BeginPt.x = cp.x + (eLength + widthTrkWnd / 2)*(ep.x - cp.x) / eLength;
		lLn.BeginPt.y = cp.y + (eLength + widthTrkWnd / 2)*(ep.y - cp.y) / eLength;
		lLn.EndPt.x = cp.x + (eLength + 4 * widthTrkWnd / 2)*(ep.x - cp.x) / eLength;
		lLn.EndPt.y = cp.y + (eLength + 4 * widthTrkWnd / 2)*(ep.y - cp.y) / eLength;
		lLn.GetLineParameter();
		lVtxBND = lLn.GetBufferVtxOfLine(length / 2);
	}

	TrackSegment() {
		m_D = 1.5;
	}
	void GetCenterOfTwoTracks() {
		m_Center = (m_Track1.CenterPt + m_Track2.CenterPt) / 2;
	}
	void GetDistanceBetweenTwoCenter() {
		double dX = m_Track1.CenterPt.x - m_Track2.CenterPt.x;
		double dY = m_Track1.CenterPt.y - m_Track2.CenterPt.y;
		m_D = sqrt(dX*dX + dY*dY);
	}
	void DuplicatePolyPair(TrackSegment _trackSeg) {
		m_Track1 = _trackSeg.m_Track1;
		m_Track2 = _trackSeg.m_Track2;
		m_D = _trackSeg.m_D;
		m_Center = _trackSeg.m_Center;
	}
	void SwapWnd(int mode) {
		if (mode == 1 || mode == 2) {
			if (m_Track1.CenterPt.x < m_Track1.CenterPt.x) {
				POINT3D pt = m_Track1.CenterPt;
				m_Track2.CenterPt = m_Track1.CenterPt;
				m_Track1.CenterPt = pt;
			}
		}
		else {
			if (m_Track1.CenterPt.x > m_Track1.CenterPt.x) {
				POINT3D pt = m_Track1.CenterPt;
				m_Track2.CenterPt = m_Track1.CenterPt;
				m_Track1.CenterPt = pt;
			}
		}
	}
};

struct SwitchBox {
	vector<POINT3D> rgn;
	POINT3D CenterPt;
	double ang;
	double referHgt;

	void GenerateRgnsUsingCenterPt(double size) {
		rgn.clear();
		vector<double> angs;
		for (int i = 0; i<4; i++)
			angs.push_back((ang + i * 90 + 45)*PI / 180);
		for (int i = 0; i<4; i++) {
			POINT3D pt(0, 0, 0);
			double dxy = size / 2 / cos(45 * PI / 180);
			pt.x = CenterPt.x + dxy*cos(angs[i]);
			pt.y = CenterPt.y + dxy*sin(angs[i]);
			rgn.push_back(pt);
		}
	}

};

struct BinaryString {
	int Type;
	double Score;
	vector<int> bStrings;
	int LabelPosTurnout;	//start: 0, end: 1
	int LabelDirection;		//left: 0, right: 1
	int LabelTurnoutType;	//merge: 0, split: 1
	int LabelPosToe;		//open: 0, close: 1, both: 2, others: -1

	BinaryString() {
		Type = -1;
		Score = 0;
		bStrings.clear();

		LabelPosTurnout = -1;
		LabelDirection = -1;
		LabelTurnoutType = -1;
		LabelPosToe = -1;
	}

	void SetBinaryString(int b1, int b2, int b3, int b4,
		int b5, int b6, int b7, int b8,
		int b9, int b10, int b11, int b12,
		int b13, int b14, int b15, int b16) {
		bStrings.clear();
		bStrings.push_back(b1);
		bStrings.push_back(b2);
		bStrings.push_back(b3);
		bStrings.push_back(b4);
		bStrings.push_back(b5);
		bStrings.push_back(b6);
		bStrings.push_back(b7);
		bStrings.push_back(b8);
		bStrings.push_back(b9);
		bStrings.push_back(b10);
		bStrings.push_back(b11);
		bStrings.push_back(b12);
		bStrings.push_back(b13);
		bStrings.push_back(b14);
		bStrings.push_back(b15);
		bStrings.push_back(b16);
	}
	void SetLables(int type) {
		//start/end
		if (type == 0 || type == 1 || type == 2 || type == 3 || type == 4 || type == 5 ||
			type == 12 || type == 13 || type == 16 || type == 17) {
			LabelPosTurnout = 0;
		}
		else
			LabelPosTurnout = 1;

		//left/right
		if (type == 1 || type == 3 || type == 5 || type == 7 || type == 9 || type == 11 ||
			type == 12 || type == 15 || type == 16 || type == 19) {
			LabelDirection = 0;
		}
		else
			LabelDirection = 1;

		//merge/split
		if (type == 2 || type == 3 || type == 6 || type == 7 || type == 10 || type == 11 ||
			type == 14 || type == 15 || type == 16 || type == 17) {
			LabelTurnoutType = 0;
		}
		else
			LabelTurnoutType = 1;

		//open/close
		if (type == 0 || type == 1 || type == 6 || type == 7)
			LabelPosToe = 0;
		else if (type == 4 || type == 5 || type == 10 || type == 11)
			LabelPosToe = 1;
		else if (type == 12 || type == 13 || type == 14 || type == 15)
			LabelPosToe = 2;
		else
			LabelPosToe = -1;
	}
};

struct TrackStack {
	//	int type;
	vector<TrackSegment> trkSegs;
	BinaryString trkBinary;

	TrackStack() {
		//		type = -1;
		trkSegs.clear();
	}
};

struct ToePt {
	POINT3D CenterPt;
	POINT3D LeftPt;
	POINT3D RightPt;
	//	Line3D ToeLn;	
	int TrkType;
	bool used;
	double TrkAng;
	int TrkMode;

	ToePt() {
		TrkType = -1;
		used = false;
	}

};

struct LoopCross {
	Line3D lc;
	double distFromSp;
};

struct PlnPara {
	double a;
	double b;
	double c;
	double d;

	PlnPara& operator=(const PlnPara &f)
	{
		if (this != &f)
		{
			a = f.a;
			b = f.b;
			c = f.c;
			d = f.d;
		}
		return *this;
	}

};
struct PtSegment {
	int SegmentID;
	vector<PointXYZDouble*> MemPts;
};

inline bool HigherZValue(const PointXYZDouble* pt1, const PointXYZDouble* pt2)
{
	return pt1->z > pt2->z ? true : false;
}
inline bool LargePts(const PtSegment s1, const PtSegment s2)
{
	return s1.MemPts.size() > s2.MemPts.size() ? true : false;
}

struct TrackLayer {
	int number;

	std::vector<TrackSegment> m_TrackSegList;
	std::vector<SwitchBox> m_SwitchBoxes;
	std::vector<LoopCross> m_LoopCross;
	std::vector<ToePt> m_ToePts;
	std::vector<TrackStack> m_TrkStks;

	std::vector<POINT3D> m_RailVec1;
	std::vector<POINT3D> m_RailVec2;
	std::vector<POINT3D> m_RailVecCenter;
	std::vector<Line3D> m_updateBnd;
	std::vector<Line3D> m_updateBnd_turnout;
	std::vector<Line3D> m_updateBnd_loop;
	std::vector<POINT3D> m_PredPts;
	std::vector<LoopCross> m_visulLoopCross;
};

struct InitialLocation {
	double x;
	double y;
	double z;
	double angle;
	int mode;

	double second_x;
	double second_y;
	double second_z;
};

struct GradeSect {
	POINT3D BeginPt;
	POINT3D EndPt;
	std::vector<POINT3D> MemVtx;
	double arc;
	double chord;
	double height;
	double grade;			//without considering curve effect
	double radius;
	double degOfcurve;
	double gradeWithcurve;	//with considering curve effect

	GradeSect() {
		BeginPt.SetPoint3D(0, 0, 0);
		EndPt.SetPoint3D(0, 0, 0);
		MemVtx.clear();
		arc = 0;
		chord = 0;
		height = 0;
		grade = 0;
		radius = 0;
		degOfcurve = 0;
		gradeWithcurve = 0;
	}

	void ResetGradeSect() {
		BeginPt.SetPoint3D(0, 0, 0);
		EndPt.SetPoint3D(0, 0, 0);
		MemVtx.clear();
		arc = 0;
		chord = 0;
		height = 0;
		grade = 0;
		radius = 0;
		degOfcurve = 0;
		gradeWithcurve = 0;
	}
};


struct ExtractionParameters {
	double trackWindowLength;
	double trackWindowWidth;
	double trackGuage;
	double minTrackPoints;
	double minPoints;
	double maxDistanceForRANSAC;
	double distanceGrouping;
	double weightParameterForMDL;
	double minIntensityRangeOfTrack;
	double maxIntensityRangeOfTrack;
};

//////////////////////////////////////////////////////////////
#endif //!define _TERRAINDEFS_H_
