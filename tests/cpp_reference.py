"""Compile original source extracts, not a second Python-derived implementation.

PCL/Windows declarations are replaced by small data shims. The only executable
source adaptation is an explicit size_t cast at the dense grid allocation to
accommodate the source's double array-length declaration in GCC. Cell arithmetic,
geometry, point membership, ordering and comparisons are unmodified.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ADE code/PCLProject/PCLProject/PCLProject"


def function(text: str, signature: str) -> str:
    # Match a definition at the start of a line, not its earlier Doxygen @fn.
    start = text.index("\n" + signature) + 1
    opening = text.index("{", start)
    depth = 0
    for position in range(opening, len(text)):
        if text[position] == "{":
            depth += 1
        elif text[position] == "}":
            depth -= 1
            if depth == 0:
                return text[start : position + 1]
    raise ValueError(f"Cannot extract {signature}")


class CppReference:
    def __init__(self):
        compiler = shutil.which("c++")
        if compiler is None or not (SOURCE / "LIDARPT.h").is_file():
            raise unittest.SkipTest("C++ compiler and original source required for parity checks")
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-cpp-")
        try:
            header = (SOURCE / "LIDARPT.h").read_text()
            extraction = (SOURCE / "RailwayExtraction.cpp").read_text()
            primitives = header[
                header.index("struct POINT3D {") : header.index("struct SwitchBox {")
            ]
            polygon = header[
                header.index("inline int pnpoly(") : header.index("inline int pnpoly2(")
            ]
            isnan = header[
                header.index("inline bool bIsNan(") : header.index("inline int FLOOR_double")
            ]
            declarations = """
struct Cloud { vector<PointXYZDouble> points; };
class RailwayExtraction {
public:
    Cloud *point_cloud;
    POINT3D m_BoundMin, m_BoundMax;
    double m_GridSize, m_NumX, m_NumY, m_NumGrid;
    double m_TrkGuage, m_gLength, m_gWidth;
    Grid *m_pGrid;
    void progressCallback(double, double) {}
    void generateGrid();
    vector<int> GetNeighborIdx(double, double, int, int, double, int);
    POINT3D ComputeFootOfPerpendicular(POINT3D, Line3D);
    TrackSegment seedGeometry(int inMode, POINT3D inPt, double inAng) {
"""
            initialization = function(extraction, "bool RailwayExtraction::Setinitiallocation(")
            seed = initialization[
                initialization.index("TrackSegment initSeg;") : initialization.index(
                    "//Assign member points"
                )
            ]
            declarations += seed + "\nreturn initSeg;\n}\n};\n"
            functions = "\n".join(
                function(extraction, signature)
                for signature in (
                    "void RailwayExtraction::generateGrid()",
                    "vector<int> RailwayExtraction::GetNeighborIdx(",
                    "POINT3D RailwayExtraction::ComputeFootOfPerpendicular(",
                )
            )
            functions = functions.replace(
                "new Grid[m_NumGrid]", "new Grid[static_cast<size_t>(m_NumGrid)]"
            )
            preamble = """
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
#define PI 3.1415926535897932
struct PointXYZDouble { double x, y, z; };
"""
            path = Path(self.directory.name) / "reference.cpp"
            path.write_text(
                preamble + isnan + polygon + primitives + declarations + functions + MAIN
            )
            self.executable = path.with_suffix("")
            result = subprocess.run(
                [compiler, "-std=c++14", "-O0", str(path), "-o", str(self.executable)],
                text=True,
                capture_output=True,
            )
            if result.returncode:
                raise RuntimeError("Original C++ extract did not compile:\n" + result.stderr)
        except BaseException:
            self.directory.cleanup()
            raise

    def close(self):
        self.directory.cleanup()

    def run(self, operation, *values):
        completed = subprocess.run(
            [str(self.executable)],
            input=operation + " " + " ".join(map(str, values)),
            text=True,
            capture_output=True,
            check=True,
        )
        payload = completed.stdout.rsplit("RESULT ", 1)[1]
        return [float(value) for value in payload.split()]


MAIN = r"""
POINT3D readPoint() {
    double x, y, z;
    cin >> x >> y >> z;
    return POINT3D(x, y, z);
}
void printPoint(POINT3D p) { cout << p.x << ' ' << p.y << ' ' << p.z << ' '; }
void printVertices(vector<POINT3D> points) { for (auto p : points) printPoint(p); }
int main() {
    cout << setprecision(17);
    string operation;
    cin >> operation;
    RailwayExtraction engine;
    if (operation == "window") {
        int mode; double angle, length, width;
        cin >> mode >> angle;
        TrackWindow window;
        window.CenterPt = readPoint();
        window.ori = angle;
        cin >> length >> width;
        window.GenerateVtxUsingCenterAndAngle(length, width, mode);
        cout << "RESULT "; printVertices(window.vtx);
    } else if (operation == "angle" || operation == "order" || operation == "side") {
        int mode = 1;
        if (operation == "order") cin >> mode;
        Line3D line;
        line.BeginPt = readPoint(); line.EndPt = readPoint();
        line.GetLineParameter(); line.GetAngle();
        cout << "RESULT ";
        if (operation == "angle") cout << line.Ang;
        else if (operation == "order") {
            line.SetEndingByMode(mode); printPoint(line.BeginPt); printPoint(line.EndPt);
        } else {
            double centerWidth, sideWidth; cin >> centerWidth >> sideWidth;
            TrackWindow window; window.GetSideVtx(line, centerWidth, sideWidth);
            printVertices(window.slVtx); printVertices(window.smVtx); printVertices(window.srVtx);
        }
    } else if (operation == "params") {
        Line3D line; double x, y; cin >> line.Ang >> x >> y;
        line.Get2DLineParameterWithAngleAndPt(x, y);
        cout << "RESULT " << line.A2D << ' ' << line.B2D << ' ' << line.C2D;
    } else if (operation == "neighbor") {
        double x, y; int nx, ny, n;
        cin >> engine.m_BoundMin.x >> engine.m_BoundMin.y >> engine.m_GridSize;
        cin >> nx >> ny >> x >> y >> n;
        auto cells = engine.GetNeighborIdx(x, y, nx, ny, engine.m_GridSize, n);
        cout << "RESULT "; for (int cell : cells) cout << cell << ' ';
    } else if (operation == "grid") {
        cin >> engine.m_GridSize;
        engine.m_BoundMin = readPoint(); engine.m_BoundMax = readPoint();
        size_t count; cin >> count;
        Cloud cloud; cloud.points.resize(count); engine.point_cloud = &cloud;
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z;
        engine.generateGrid();
        vector<int> cells(count, -1);
        for (int cell = 0; cell < engine.m_NumGrid; ++cell)
            for (auto p : engine.m_pGrid[cell].MemPnts) cells[p - cloud.points.data()] = cell;
        cout << "\nRESULT " << engine.m_NumX << ' ' << engine.m_NumY << ' ';
        for (int cell : cells) cout << cell << ' ';
        delete[] engine.m_pGrid;
    } else if (operation == "polygon") {
        int n, count; cin >> n >> count;
        vector<double> x(n), y(n);
        for (int i = 0; i < n; ++i) cin >> x[i] >> y[i];
        cout << "RESULT ";
        for (int i = 0; i < count; ++i) {
            double px, py; cin >> px >> py;
            cout << pnpoly(n, x.data(), y.data(), px, py) << ' ';
        }
    } else if (operation == "seed") {
        int mode; double angle; cin >> mode >> angle;
        POINT3D point = readPoint();
        cin >> engine.m_TrkGuage >> engine.m_gLength >> engine.m_gWidth;
        auto segment = engine.seedGeometry(mode, point, angle);
        cout << "RESULT ";
        printPoint(segment.m_Track1.CenterPt); printPoint(segment.m_Track2.CenterPt);
        printVertices(segment.m_Track1.vtx); printVertices(segment.m_Track2.vtx);
    } else if (operation == "foot") {
        POINT3D p = readPoint(); Line3D line;
        line.BeginPt = readPoint(); line.EndPt = readPoint(); line.GetLineParameter();
        cout << "RESULT "; printPoint(engine.ComputeFootOfPerpendicular(p, line));
    } else return 2;
    cout << '\n';
    return 0;
}
"""
