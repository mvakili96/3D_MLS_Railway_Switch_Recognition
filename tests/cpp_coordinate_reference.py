"""Compile original UTM.cpp and all DataConvertor bodies with PCL data shims.

No projection formulas or converter loops are reimplemented in the oracle.
Callbacks copy reached point state. Windows dialogs and batch UI are excluded.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.cpp_reference import SOURCE


PREAMBLE = r"""
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "UTM.h"
using namespace std;
#define PI 3.1415926535897932
struct PointXYZDouble {double x=0,y=0,z=0;int ClassCode=0,SegmentID=-1,Intensity=0;};
namespace pcl {template<class T> struct PointCloud {using Ptr=shared_ptr<PointCloud<T>>;vector<T> points;};}
using UnmanagedProgressCallback=void(*)(double,double);
class DataConvertor {
 public:
 pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;vector<PointXYZDouble> convertedPts;
 double a,e2,a1,a2,a3,a4,a5,a6;
 DataConvertor(pcl::PointCloud<PointXYZDouble>::Ptr);~DataConvertor();
 void ConvertECEFToUTM(int);void ConvertECEFToUTM(int,UnmanagedProgressCallback);
 void ConvertUTMToECEF(int,bool);void ConvertUTMToECEF(int,bool,UnmanagedProgressCallback);
 void Ecef_to_geodetic(double,double,double,double&,double&,double&);
 void Geodetic_to_ecef(double,double,double,double&,double&,double&);
 void SaveConvertedPoints(const char*);void SaveConvertedPoints(const char*,UnmanagedProgressCallback);
 void SaveConvertedPoints(const char*,UnmanagedProgressCallback,const char*);
};
"""

MAIN = r"""
double scalar(){string s;cin>>s;return stod(s);}
PointXYZDouble readPoint(){PointXYZDouble p;p.x=scalar();p.y=scalar();p.z=scalar();cin>>p.ClassCode>>p.SegmentID>>p.Intensity;return p;}
vector<PointXYZDouble> readPoints(){size_t n;cin>>n;vector<PointXYZDouble> p(n);for(auto &v:p)v=readPoint();return p;}
struct Event {double current,total;vector<PointXYZDouble> points;};
vector<Event> events;DataConvertor* active;
void callback(double current,double total){events.push_back({current,total,active->convertedPts});}
void printPoints(vector<PointXYZDouble> points){cout<<points.size()<<' ';for(auto p:points)cout<<p.x<<' '<<p.y<<' '<<p.z<<' '<<p.ClassCode<<' '<<p.SegmentID<<' '<<p.Intensity<<' ';}
void printEvents(vector<Event> items){cout<<items.size()<<' ';for(auto e:items){cout<<e.current<<' '<<e.total<<' ';printPoints(e.points);}}
int main(int argc,char**argv){cout<<setprecision(17);string op;cin>>op;
 auto cloud=make_shared<pcl::PointCloud<PointXYZDouble>>();DataConvertor converter(cloud);active=&converter;
 double x,y,z;
 if(op=="convert"){
  cloud->points=readPoints();converter.convertedPts=readPoints();size_t n;cin>>n;
  vector<pair<vector<PointXYZDouble>,vector<Event>>> results;
  for(size_t i=0;i<n;++i){string direction;int zone;bool south,progress;cin>>direction>>zone>>south>>progress;events.clear();
   if(direction=="ecef_to_utm"){if(progress)converter.ConvertECEFToUTM(zone,callback);else converter.ConvertECEFToUTM(zone);}
   else {if(progress)converter.ConvertUTMToECEF(zone,south,callback);else converter.ConvertUTMToECEF(zone,south);}
   results.push_back({converter.convertedPts,events});}
  cout<<"RESULT "<<results.size()<<' ';for(auto result:results){printPoints(result.first);printEvents(result.second);}return 0;
 }
 if(op=="export"){
  converter.convertedPts=readPoints();
  if(string(argv[3])=="0")converter.SaveConvertedPoints(argv[1]);
  else if(string(argv[3])=="1")converter.SaveConvertedPoints(argv[1],callback);
  else converter.SaveConvertedPoints(argv[1],callback,argv[2]);
  cout<<"RESULT ";printEvents(events);return 0;
 }
 cout<<"RESULT ";
 if(op=="deg")cout<<DegToRad(scalar());
 else if(op=="rad")cout<<RadToDeg(scalar());
 else if(op=="arc")cout<<ArcLengthOfMeridian(scalar());
 else if(op=="central"){int zone;cin>>zone;cout<<UTMCentralMeridian(zone);}
 else if(op=="foot")cout<<FootpointLatitude(scalar());
 else if(op=="map"){double a=scalar(),b=scalar(),c=scalar();MapLatLonToXY(a,b,c,x,y);cout<<x<<' '<<y;}
 else if(op=="unmap"){double a=scalar(),b=scalar(),c=scalar();MapXYToLatLon(a,b,c,x,y);cout<<x<<' '<<y;}
 else if(op=="utm"){double a=scalar(),b=scalar();int zone;cin>>zone;zone=LatLonToUTMXY(a,b,zone,x,y);cout<<zone<<' '<<x<<' '<<y;}
 else if(op=="unutm"){double a=scalar(),b=scalar();int zone;bool south;cin>>zone>>south;UTMXYToLatLon(a,b,zone,south,x,y);cout<<x<<' '<<y;}
 else if(op=="ecef_geo"){double a=scalar(),b=scalar(),c=scalar();converter.Ecef_to_geodetic(a,b,c,x,y,z);cout<<x<<' '<<y<<' '<<z;}
 else if(op=="geo_ecef"){double a=scalar(),b=scalar(),c=scalar();converter.Geodetic_to_ecef(a,b,c,x,y,z);cout<<x<<' '<<y<<' '<<z;}
}
"""


class CppCoordinateReference:
    def __init__(self):
        compiler = shutil.which("c++") or shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-coordinate-cpp-")
        try:
            converter = (SOURCE / "DataConvertor.cpp").read_text()
            converter = "\n".join(
                line for line in converter.splitlines() if not line.startswith("#include")
            )
            path = Path(self.directory.name) / "reference.cpp"
            path.write_text(PREAMBLE + (SOURCE / "UTM.cpp").read_text() + converter + MAIN)
            self.executable = path.with_suffix("")
            result = subprocess.run(
                [
                    compiler,
                    "-std=c++14",
                    "-O0",
                    "-I",
                    str(SOURCE),
                    str(path),
                    "-o",
                    str(self.executable),
                ],
                text=True,
                capture_output=True,
            )
            if result.returncode:
                raise RuntimeError(result.stderr)
        except BaseException:
            self.close()
            raise

    def close(self):
        self.directory.cleanup()

    def run(self, operation, *values, args=()):
        result = subprocess.run(
            [str(self.executable), *map(str, args)],
            input=operation + " " + " ".join(map(str, values)),
            capture_output=True,
            text=True,
            check=True,
        )
        return [float(v) for v in result.stdout.rsplit("RESULT ", 1)[1].split()]

    def export(self, points, delimiter=" ", overload=2):
        values = [
            len(points),
            *(v for p in points for v in (p.x, p.y, p.z, p.class_code, p.segment_id, p.intensity)),
        ]
        path = Path(self.directory.name) / "export.pts"
        events = self.run("export", *values, args=(path, delimiter, overload))
        return path.read_text(), events
