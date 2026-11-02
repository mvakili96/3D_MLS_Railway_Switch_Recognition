"""Run frozen original PCL bodies and LAS helpers against the bundled Eigen.

Point/container/field-trait shims cover PointXYZ only. The actual accumulator,
centroid, bounds, filter, voxel sort/centroid and affine arithmetic are compiled.
Unused filter branches throw if reached. A prefix stops before unsafe casts.
"""

import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from tests.cpp_reference import SOURCE, function


ROOT = Path(__file__).resolve().parents[1]
EIGEN = ROOT / "ADE code/Installations/Installations/PCL 1.8.0/3rdParty/Eigen/eigen3"

PREAMBLE = r"""
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>
#define PCL_WARN(...) ((void)0)
#define PCL_ERROR(...) ((void)0)
#define pcl_isfinite(x) std::isfinite(x)
using std::vector;using std::string;using std::shared_ptr;using std::make_shared;
using std::runtime_error;using std::isfinite;using std::cout;using std::cin;
using std::setprecision;using std::stod;
namespace boost { namespace mpl {struct _1{};}
namespace fusion {template<class T,class F> void for_each(T&t,F f){f(std::get<0>(t));}}}
namespace pcl {
static_assert(std::is_same<decltype(floor(0.1f)),double>::value,"preserve unqualified source floor; do not import std::floor");
namespace traits {template<class T>struct has_xyz{};}
struct alignas(16) PointXYZ {
 union {float data[4];struct{float x,y,z;};};
 PointXYZ(){x=y=z=0;data[3]=1;}
 Eigen::Map<Eigen::Vector3f> getVector3fMap(){return Eigen::Map<Eigen::Vector3f>(data);}
 Eigen::Map<const Eigen::Vector3f> getVector3fMap()const{return Eigen::Map<const Eigen::Vector3f>(data);}
 Eigen::Map<Eigen::Vector4f> getVector4fMap(){return Eigen::Map<Eigen::Vector4f>(data);}
 Eigen::Map<const Eigen::Vector4f> getVector4fMap()const{return Eigen::Map<const Eigen::Vector4f>(data);}
 Eigen::Map<const Eigen::Array4f> getArray4fMap()const{return Eigen::Map<const Eigen::Array4f>(data);}
};
using Array4fMapConst=Eigen::Map<const Eigen::Array4f>;
struct Header {uint32_t seq=0;uint64_t stamp=0;string frame_id;};
template<class T>struct PointCloud {
 using Ptr=shared_ptr<PointCloud<T>>;using ConstPtr=shared_ptr<const PointCloud<T>>;
 vector<T> points;uint32_t width=0,height=0;bool is_dense=true;Header header;
 Eigen::Vector4f sensor_origin_=Eigen::Vector4f::Zero();Eigen::Quaternionf sensor_orientation_=Eigen::Quaternionf::Identity();
 size_t size()const{return points.size();}bool empty()const{return points.empty();}
 T&operator[](size_t i){return points[i];}const T&operator[](size_t i)const{return points[i];}
};
template<class T>bool isFinite(const T&p){return isfinite(p.x)&&isfinite(p.y)&&isfinite(p.z);}
template<class T>void copyPointCloud(const PointCloud<T>&a,PointCloud<T>&b){b=a;}
struct PCLPointField{size_t offset=0;};
template<class T>int getFieldIndex(const PointCloud<T>&,string,vector<PCLPointField>&){throw runtime_error("unused field filter");}
struct PCLException:runtime_error {PCLException(const char*s,const char*,const char*):runtime_error(s){}};
template<class T>void getMinMax3D(const typename PointCloud<T>::ConstPtr&,const vector<int>&,string,float,float,Eigen::Vector4f&,Eigen::Vector4f&,bool){throw runtime_error("unused field bounds");}
template<class T,class S>unsigned int compute3DCentroid(const PointCloud<T>&,Eigen::Matrix<S,4,1>&);
template<class T,class S>void transformPointCloud(const PointCloud<T>&,PointCloud<T>&,const Eigen::Transform<S,3,Eigen::Affine>&,bool=true);
namespace detail {
"""

VOXEL = r"""
template<class PointT>class VoxelGrid {
 using PointCloud=pcl::PointCloud<PointT>;
 typename PointCloud::ConstPtr input_;shared_ptr<vector<int>> indices_;bool fake_indices_=false;
 Eigen::Vector4f leaf_size_;Eigen::Array4f inverse_leaf_size_;bool downsample_all_data_,save_leaf_layout_;
 vector<int>leaf_layout_;Eigen::Vector4i min_b_,max_b_,div_b_,divb_mul_;
 string filter_field_name_,filter_name_;double filter_limit_min_,filter_limit_max_;bool filter_limit_negative_;unsigned min_points_per_voxel_;
 string getClassName(){return filter_name_;}
 public:
 Eigen::Vector4f lastMin,lastMax;vector<int64_t>lastCounts;vector<int>lastBins,lastSorted;bool lastFallback=false;
 void setInputCloud(typename PointCloud::ConstPtr p){input_=p;}
 bool initCompute();bool deinitCompute();void applyFilter(PointCloud&);void applyPrefix(PointCloud&);
"""

MAIN = r"""
double readScalar(){string s;cin>>s;return stod(s);}
void printCloud(const pcl::PointCloud<pcl::PointXYZ>&p,const pcl::PointCloud<pcl::PointXYZ>&input){
 cout<<p.points.size()<<' '<<p.width<<' '<<p.height<<' '<<p.is_dense<<' '<<p.header.seq<<' '<<p.header.stamp<<' '<<(p.header.frame_id==input.header.frame_id)<<' ';
 for(int i=0;i<4;i++)cout<<p.sensor_origin_[i]<<' ';for(int i=0;i<4;i++)cout<<p.sensor_orientation_.coeffs()[i]<<' ';
 for(auto point:p.points)cout<<point.x<<' '<<point.y<<' '<<point.z<<' ';
}
int main(){cout<<setprecision(17);string op;cin>>op;size_t n;cin>>n;
 auto cloud=make_shared<pcl::PointCloud<pcl::PointXYZ>>();cin>>cloud->is_dense>>cloud->width>>cloud->height>>cloud->header.seq>>cloud->header.stamp;cloud->header.frame_id="native-frame";
 for(int i=0;i<4;i++)cloud->sensor_origin_[i]=readScalar();for(int i=0;i<4;i++)cloud->sensor_orientation_.coeffs()[i]=readScalar();
 cloud->points.resize(n);for(auto &p:cloud->points){p.x=readScalar();p.y=readScalar();p.z=readScalar();}
 RailwayLasHelpers helper;
 if(op=="center"){
  auto output=helper.Transform(cloud);cout<<"RESULT "<<lastCentroidCount<<' ';for(int i=0;i<4;i++)cout<<lastCentroid[i]<<' ';
  for(int i=0;i<4;i++)for(int j=0;j<4;j++)cout<<lastTransform(i,j)<<' ';printCloud(*output,*cloud);
 }else{
  pcl::VoxelGrid<pcl::PointXYZ>filter;filter.setInputCloud(cloud);filter.setLeafSize(.1f,.1f,.1f);pcl::PointCloud<pcl::PointXYZ>output;
  if(op=="prefix")filter.filterPrefix(output);else output=*helper.DecreaseDensity(cloud, &filter);
  cout<<"RESULT ";for(int i=0;i<3;i++)cout<<filter.lastMin[i]<<' ';for(int i=0;i<3;i++)cout<<filter.lastMax[i]<<' ';
  cout<<filter.lastCounts.size()<<' ';for(auto v:filter.lastCounts)cout<<v<<' ';
  cout<<filter.lastBins.size()<<' ';for(auto v:filter.lastBins)cout<<v<<' ';
  cout<<filter.lastSorted.size()<<' ';for(auto v:filter.lastSorted)cout<<v<<' ';
  cout<<filter.lastFallback<<' ';printCloud(output,*cloud);
 }
}
"""


class CppPclReference:
    def __init__(self):
        compiler = shutil.which("c++") or shutil.which("g++")
        if compiler is None or not (EIGEN / "Eigen/Core").is_file():
            raise unittest.SkipTest(
                "PCL comparison requires a C++ compiler and the bundled Eigen headers"
            )
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-pcl-cpp-")
        try:
            payload = json.loads((ROOT / "src/railway_mls/pcl_reference.json").read_text())
            code = {k: v["code"] for k, v in payload["extracts"].items()}
            preamble = PREAMBLE + code["accumulator_xyz"] + ";\n"
            preamble += "template<class T,class O=T>struct Accumulators {using type=std::tuple<AccumulatorXYZ>;};\n"
            preamble += "template<class PointT>" + code["add_point"] + ";\n"
            preamble += "template<class PointT>" + code["get_point"] + ";\n}\n"
            preamble += "template<class PointT>" + code["centroid_point"] + ";\n"
            preamble += "template<class PointT>inline void\n" + code["bounds"] + "\n"
            preamble += (
                "template<class PointT,class Scalar>inline unsigned int\n" + code["centroid"] + "\n"
            )
            transform = code["transform"]
            preamble += "template<class PointT,class Scalar>void\n" + transform + "\n"
            voxel = (
                VOXEL
                + code["voxel_constructor"]
                + "\nvoid "
                + code["voxel_leaf"]
                + "\nvoid "
                + code["filter"]
                + "\n"
            )
            voxel += (
                "void "
                + code["filter"]
                .replace("filter (PointCloud &output)", "filterPrefix (PointCloud &output)", 1)
                .replace("applyFilter (", "applyPrefix (")
                + "\n};\n}\n"
            )
            functions = ""
            for name in ("init_compute", "deinit_compute"):
                functions += (
                    "template<class PointT>bool\n"
                    + code[name].replace("pcl::PCLBase<PointT>", "pcl::VoxelGrid<PointT>")
                    + "\n"
                )
            apply = code["voxel_apply"]
            marker = "getMinMax3D<PointT> (*input_, *indices_, min_p, max_p);"
            assert marker in apply
            apply = apply.replace(marker, marker + "\nlastMin=min_p;lastMax=max_p;")
            prefix = apply[: apply.index("// Check that the leaf size")] + "\n}\n"
            functions += code["voxel_index"] + ";\n"
            marker = "int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;"
            assert marker in apply
            apply = apply.replace(marker, marker + "\nlastCounts={dx,dy,dz};")
            apply = apply.replace("output = *input_;", "lastFallback=true;output = *input_;")
            apply = apply.replace(
                "div_b_[3] = 0;",
                "div_b_[3] = 0;lastBins={min_b_[0],min_b_[1],min_b_[2],max_b_[0],max_b_[1],max_b_[2],div_b_[0],div_b_[1],div_b_[2]};",
            )
            marker = "std::sort (index_vector.begin (), index_vector.end (), std::less<cloud_point_index_idx> ());"
            assert marker in apply
            apply = apply.replace(
                marker,
                marker + "\nfor(auto p:index_vector)lastSorted.push_back(p.cloud_point_index);",
            )
            functions += (
                "template<class PointT>void\n"
                + apply
                + "\ntemplate<class PointT>void\n"
                + prefix.replace("::applyFilter", "::applyPrefix", 1)
                + "\n"
            )
            native = (SOURCE / "LASFileReader.cpp").read_text()
            density = function(
                native, "pcl::PointCloud<pcl::PointXYZ>::Ptr LASFileReader::DecreaseDensity("
            )
            # Route the diagnostic output to the same original filter object.
            density = density.replace(
                "LASFileReader::DecreaseDensity(", "RailwayLasHelpers::DecreaseDensity(", 1
            )
            density = density.replace(
                "Ptr cloud)", "Ptr cloud, pcl::VoxelGrid<pcl::PointXYZ>*diagnostic)", 1
            )
            density = density.replace(
                "pcl::VoxelGrid<pcl::PointXYZ> sor;",
                "pcl::VoxelGrid<pcl::PointXYZ>& sor=*diagnostic;",
            )
            center = function(
                native, "pcl::PointCloud<pcl::PointXYZ>::Ptr LASFileReader::Transform("
            ).replace("LASFileReader::", "RailwayLasHelpers::", 1)
            center = center.replace(
                "pcl::compute3DCentroid(*cloud, centroid);",
                "lastCentroidCount=pcl::compute3DCentroid(*cloud, centroid);lastCentroid=centroid;",
            )
            marker = (
                "transform_2.translation() << -1 * centroid[0], -1 * centroid[1], -1 * centroid[2];"
            )
            assert marker in center
            center = center.replace(marker, marker + "\nlastTransform=transform_2.matrix();")
            declarations = """unsigned lastCentroidCount=0;Eigen::Vector4d lastCentroid;Eigen::Matrix4d lastTransform;
class RailwayLasHelpers {public:pcl::PointCloud<pcl::PointXYZ>::Ptr DecreaseDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::VoxelGrid<pcl::PointXYZ>*);pcl::PointCloud<pcl::PointXYZ>::Ptr Transform(pcl::PointCloud<pcl::PointXYZ>::Ptr);};\n"""
            path = Path(self.directory.name) / "reference.cpp"
            path.write_text(preamble + voxel + functions + declarations + density + center + MAIN)
            self.executable = path.with_suffix("")
            result = subprocess.run(
                [
                    compiler,
                    "-std=c++14",
                    "-O0",
                    "-w",
                    "-Wno-deprecated-declarations",
                    "-I",
                    str(EIGEN),
                    str(path),
                    "-o",
                    str(self.executable),
                ],
                capture_output=True,
                text=True,
            )
            if result.returncode:
                raise RuntimeError(result.stderr[-14000:])
        except BaseException:
            self.close()
            raise

    def close(self):
        self.directory.cleanup()

    def run(self, operation, cloud):
        tokens = [
            operation,
            len(cloud),
            int(cloud.is_dense),
            cloud.width,
            cloud.height,
            cloud.header.sequence,
            cloud.header.stamp,
            *cloud.sensor_origin,
            *cloud.sensor_orientation,
            *cloud.xyz.reshape(-1),
        ]
        result = subprocess.run(
            [str(self.executable)],
            input=" ".join(map(str, tokens)),
            capture_output=True,
            text=True,
            check=True,
        )
        return [float(v) for v in result.stdout.rsplit("RESULT ", 1)[1].split()]
