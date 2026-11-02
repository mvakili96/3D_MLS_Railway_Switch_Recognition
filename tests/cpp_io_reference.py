"""Compile original reader/export bodies with PCL/libLAS data/decoder shims.

The LAS shim receives already-decoded records. It tests source field assignment,
alternating selection, header bounds and callbacks, not libLAS codec equivalence.
MIN/MAX are conditional comparison shims; the historical macro inclusion chain
is unverified, so equal signed-zero bounds are outside the native parity claim.
Actual file-format decoding is tested separately with generated LAS/LAZ files.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.cpp_reference import SOURCE, function


PREAMBLE = r"""
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
using namespace std;
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define RAIL_INTENSITY 100
#define RAIL_INTENSITY_LEFT 100
#define RAIL_INTENSITY_RIGHT 200
struct PointXYZDouble {
 double x=0,y=0,z=0;int ClassCode=0,SegmentID=-1,Intensity=0;
 PointXYZDouble(){} PointXYZDouble(double a,double b,double c):x(a),y(b),z(c){}
};
struct POINT3D {double x=0,y=0,z=0;int PtType=0;};
namespace pcl {template<class T> struct PointCloud {
 using Ptr=shared_ptr<PointCloud<T>>;vector<T> points;
};}
using UnmanagedProgressCallback=void(*)(double,double);
namespace liblas {
struct Extent {double values[6];double minx(){return values[0];}double miny(){return values[1];}
 double minz(){return values[2];}double maxx(){return values[3];}double maxy(){return values[4];}double maxz(){return values[5];}};
struct Header {size_t count;Extent extent;size_t GetPointRecordsCount(){return count;}Extent GetExtent(){return extent;}};
struct Point {double x,y,z;unsigned short intensity;
 double GetX()const{return x;}double GetY()const{return y;}double GetZ()const{return z;}
 unsigned short GetIntensity()const{return intensity;}};
struct Reader {Header header;vector<Point> points;size_t cursor=0;
 Reader(istream &s){s>>header.count;for(auto &v:header.extent.values)s>>v;
  for(size_t i=0;i<header.count;++i){Point p;s>>p.x>>p.y>>p.z>>p.intensity;points.push_back(p);}}
 Header GetHeader(){return header;}bool ReadNextPoint(){return cursor<points.size()?++cursor,true:false;}
 Point const& GetPoint(){return points[cursor-1];}};
struct ReaderFactory {Reader CreateWithStream(istream &s){return Reader(s);}};
}
class TXTFileReader {
 pcl::PointCloud<PointXYZDouble>::Ptr loaded_point_cloud;
 public: PointXYZDouble minPt,maxPt;
 TXTFileReader();~TXTFileReader();
 pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char*);
 pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char*,UnmanagedProgressCallback);
 pcl::PointCloud<PointXYZDouble>::Ptr ReadDataCSV(const char*,UnmanagedProgressCallback);
 pcl::PointCloud<PointXYZDouble>::Ptr LoadedData();
 void saveRailVector(const char*,vector<POINT3D>);
 void saveRailVector(const char*,vector<POINT3D>,vector<POINT3D>,vector<POINT3D>);
 void saveRailVectorWithIndex(const char*,vector<POINT3D>,vector<POINT3D>,vector<POINT3D>);
};
class LASFileReader {
 pcl::PointCloud<PointXYZDouble>::Ptr loaded_point_cloud;
 public: LASFileReader();~LASFileReader();
 pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char*);
 pcl::PointCloud<PointXYZDouble>::Ptr ReadData(const char*,UnmanagedProgressCallback);
 pcl::PointCloud<PointXYZDouble>::Ptr LoadedData();
 PointXYZDouble maxPoint(const char*);PointXYZDouble minPoint(const char*);
};
class RailwayExtraction {
 public:PointXYZDouble m_BoundMin,m_BoundMax;
 void updateMaxMin(PointXYZDouble,PointXYZDouble);
 void updateMaxMinFromReader(LASFileReader*,const char*);
 void readMaxMinFromReader(LASFileReader*,const char*);
 void readMaxMinFromTXTReader(TXTFileReader*);
};
"""

MAIN = r"""
vector<pair<double,double>> events;
void callback(double current,double total){events.push_back({current,total});}
void printPoint(PointXYZDouble p){cout<<p.x<<' '<<p.y<<' '<<p.z<<' ';}
vector<POINT3D> readVector(istream &s){size_t n;s>>n;vector<POINT3D> out(n);
 for(auto &p:out)s>>p.x>>p.y>>p.z>>p.PtType;return out;}
int main(int argc,char**argv){cout<<setprecision(17);string op=argv[1];
 if(op=="center"||op=="paired"||op=="indexed"){
  ifstream data(argv[2]);auto c=readVector(data),l=readVector(data),r=readVector(data);TXTFileReader reader;
  if(op=="center")reader.saveRailVector(argv[3],c);
  else if(op=="paired")reader.saveRailVector(argv[3],c,l,r);
  else reader.saveRailVectorWithIndex(argv[3],c,l,r);
  return 0;
 }
 pcl::PointCloud<PointXYZDouble>::Ptr cloud;RailwayExtraction engine;bool progress=atoi(argv[3]);
 if(op=="las"){LASFileReader reader;cloud=progress?reader.ReadData(argv[2],callback):reader.ReadData(argv[2]);
  engine.updateMaxMinFromReader(&reader,argv[2]);
  if(cloud!=reader.LoadedData())return 2;
 }else{TXTFileReader reader;
  if(op=="csv")cloud=reader.ReadDataCSV(argv[2],callback);
  else cloud=progress?reader.ReadData(argv[2],callback):reader.ReadData(argv[2]);
  engine.readMaxMinFromTXTReader(&reader);if(cloud!=reader.LoadedData())return 2;
 }
 cout<<cloud->points.size()<<' ';printPoint(engine.m_BoundMin);printPoint(engine.m_BoundMax);
 for(auto p:cloud->points){printPoint(p);cout<<p.Intensity<<' '<<p.ClassCode<<' '<<p.SegmentID<<' ';}
 cout<<events.size()<<' ';for(auto pair:events)cout<<pair.first<<' '<<pair.second<<' ';
}
"""


class CppIOReference:
    def __init__(self):
        compiler = shutil.which("c++") or shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable")
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-io-cpp-")
        try:
            text = (SOURCE / "TXTFileReader.cpp").read_text()
            las = (SOURCE / "LASFileReader.cpp").read_text()
            engine = (SOURCE / "RailwayExtraction.cpp").read_text()
            # Removing includes retains every original reader/export body.
            text = "\n".join(line for line in text.splitlines() if not line.startswith("#"))
            signatures = (
                "LASFileReader::LASFileReader()",
                "LASFileReader::~LASFileReader()",
                "pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::ReadData(const char *fileName)",
                "pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::ReadData(const char * fileName, UnmanagedProgressCallback callback)",
                "pcl::PointCloud<PointXYZDouble>::Ptr LASFileReader::LoadedData()",
                "PointXYZDouble LASFileReader::maxPoint(",
                "PointXYZDouble LASFileReader::minPoint(",
            )
            bodies = "\n".join(function(las, signature) for signature in signatures)
            bodies += "\n" + "\n".join(
                function(engine, signature)
                for signature in (
                    "void RailwayExtraction::updateMaxMin(PointXYZDouble",
                    "void RailwayExtraction::updateMaxMinFromReader(",
                    "void RailwayExtraction::readMaxMinFromReader(",
                    "void RailwayExtraction::readMaxMinFromTXTReader(",
                )
            )
            source = Path(self.directory.name) / "reference.cpp"
            source.write_text(PREAMBLE + text + bodies + MAIN)
            self.executable = source.with_suffix("")
            result = subprocess.run(
                [compiler, "-std=c++14", "-O0", str(source), "-o", str(self.executable)],
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

    def read(self, format, payload, callback=True):
        path = Path(self.directory.name) / "input.txt"
        path.write_text(payload)
        output = subprocess.run(
            [str(self.executable), format, str(path), str(int(callback))],
            check=True,
            text=True,
            capture_output=True,
        )
        return [float(v) for v in output.stdout.split()]

    def export(self, mode, center, left=(), right=()):
        path = Path(self.directory.name) / "vectors.txt"
        tokens = []
        for vector in (center, left, right):
            tokens.extend(
                (len(vector), *(v for p in vector for v in (*p.as_array(), p.point_type)))
            )
        path.write_text(" ".join(map(str, tokens)))
        output = path.with_suffix(".csv")
        subprocess.run(
            [str(self.executable), mode, str(path), str(output)], check=True, capture_output=True
        )
        return output.read_text()
