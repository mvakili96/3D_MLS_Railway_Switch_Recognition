"""Original scalar point/parameter bodies and active seed-form normalization.

PCL registration/alignment are removed from the point header. The two parameter
method bodies and their member types are extracted unchanged. In the form body,
only XYZ Text assignments are omitted; small numeric controls replace Windows
controls. No undefined float-to-int casts or uninitialized fields are executed.
"""

from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from tests.cpp_reference import SOURCE, function


class CppBoundaryReference:
    def __init__(self):
        compiler = shutil.which("c++")
        if compiler is None or not (SOURCE / "PointXYZDouble.h").is_file():
            raise unittest.SkipTest("original source and C++ compiler required")
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-boundary-cpp-")
        try:
            point = (SOURCE / "PointXYZDouble.h").read_text()
            point = point[point.index("struct PointXYZDouble"):point.index("POINT_CLOUD_REGISTER")]
            header = (SOURCE / "LIDARPT.h").read_text()
            parameters = header[header.index("struct ExtractionParameters"):].split("};", 1)[0] + "};"
            location = header[header.index("struct InitialLocation"):].split("};", 1)[0] + "};"
            engine_header = (SOURCE / "RailwayExtraction.h").read_text()
            members = engine_header[engine_header.index("\tdouble m_GridSize;"):
                                    engine_header.index("\tstd::vector<SwitchBox>")]
            engine = (SOURCE / "RailwayExtraction.cpp").read_text()
            methods = function(engine, "void RailwayExtraction::updateExtractionParameters(")
            methods += function(engine, "ExtractionParameters RailwayExtraction::getExtractionParameters(")
            form = function((SOURCE / "InitialLocationForm.cpp").read_text(),
                            "System::Void PCLProject::InitialLocationForm::updateInputs(InitialLocation location)")
            form = "\n".join(line for line in form.splitlines() if "->Text =" not in line)
            form = form.replace("System::Void", "void")
            fields = re.findall(r"\bdouble\s+(\w+)\s*;", parameters)
            reader = "".join(f"cin >> input.{field};" for field in fields)
            printer = "".join(f"cout << output.{field} << ' ';" for field in fields)
            main = r"""
int main(){cout<<setprecision(17);string op;cin>>op;
 if(op=="parameters"){ExtractionParameters input;READ
  RailwayExtraction engine;engine.m_GridSize=2.5;engine.m_skip_rate=3;
  engine.updateExtractionParameters(input);auto output=engine.getExtractionParameters();
  cout<<"RESULT "<<engine.m_GridSize<<' '<<engine.m_skip_rate<<' ';PRINT
 }else if(op=="point"){
  PointXYZDouble a;cin>>a.x>>a.y>>a.z>>a.ClassCode>>a.SegmentID>>a.Intensity;
  PointXYZDouble b(a.x,a.y,a.z);PointXYZDouble c(&a),d;
  cout<<"RESULT "<<b.data[3]<<' '<<b.ClassCode<<' '<<b.SegmentID<<' '<<b.Intensity<<' '
   <<c.data[3]<<' '<<c.x<<' '<<c.y<<' '<<c.z<<' '<<c.ClassCode<<' '<<c.SegmentID<<' '<<c.Intensity<<' '
   <<d.x<<' '<<d.y<<' '<<d.z<<' '<<d.data[3]<<' '<<d.ClassCode<<' '<<d.SegmentID<<' '<<d.Intensity<<' '
   <<a.isNull();
 }else if(op=="equal"){
  PointXYZDouble a,b;cin>>a.x>>a.y>>a.z>>b.x>>b.y>>b.z;
  a.ClassCode=3;b.ClassCode=7;cout<<"RESULT "<<(a==b)<<' '<<a.IsEqual(b)<<' '<<a.isNull();
 }else if(op=="seed"){
  InitialLocation input;cin>>input.angle>>input.mode;
  PCLProject::InitialLocationForm form;form.updateInputs(input);
  cout<<"RESULT "<<form.angle.Value<<' '<<form.mode.Value;
 }
}
""".replace("READ", reader).replace("PRINT", printer)
            preamble = """
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
using namespace std;
#define EIGEN_ALIGN16
struct Grid {};
"""
            form_class = """
namespace PCLProject {class InitialLocationForm {public:
 struct Control {double Value;};Control angle,mode;Control *angleUpDown=&angle,*modeUpDown=&mode;
 void updateInputs(InitialLocation location);
};}
"""
            path = Path(self.directory.name) / "reference.cpp"
            path.write_text(preamble + point + parameters + location + "class RailwayExtraction{public:"
                            + members + "void updateExtractionParameters(ExtractionParameters);"
                            + "ExtractionParameters getExtractionParameters();};"
                            + methods + form_class + form + main)
            self.executable = path.with_suffix("")
            result = subprocess.run([compiler, "-std=c++14", "-O0", str(path), "-o",
                                     str(self.executable)], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stderr)
        except BaseException:
            self.close()
            raise

    def close(self):
        self.directory.cleanup()

    def run(self, operation, *values):
        result = subprocess.run([str(self.executable)],
                                input=operation + " " + " ".join(map(str, values)),
                                capture_output=True, text=True, check=True)
        return [float(value) for value in result.stdout.split("RESULT ", 1)[1].split()]
