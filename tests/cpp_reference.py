"""Compile original source extracts, not a second Python-derived implementation.

PCL/Windows declarations are replaced by small data shims. An allocation-size cast
accommodates the source's double array length in GCC. For stochastic comparisons,
rand() reads a supplied raw-draw stream instead of the host CRT generator. Tested
arithmetic, branches and thresholds remain unchanged. MDL diagnostic assignments
capture its existing local scores/flags without changing the selection rules.
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
            higher_z = function(header, "inline bool HigherZValue(")
            segments = header[
                header.index("struct PtSegment {") : header.index("inline bool HigherZValue(")
            ]
            larger = function(header, "inline bool LargePts(")
            declarations = """
struct Cloud { vector<PointXYZDouble> points; };
class RailwayExtraction {
public:
    Cloud *point_cloud;
    POINT3D m_BoundMin, m_BoundMax;
    double m_GridSize, m_NumX, m_NumY, m_NumGrid;
    double m_TrkGuage, m_gLength, m_gWidth;
    double m_ThRansac, m_ThDistGrouping, m_wMDL;
    int m_skip_rate;
    vector<double> lastMDLScores, lastMDLCloseness;
    vector<int> lastMDLFlags;
    Grid *m_pGrid;
    void progressCallback(double, double) {}
    void generateGrid();
    vector<int> GetNeighborIdx(double, double, int, int, double, int);
    POINT3D ComputeFootOfPerpendicular(POINT3D, Line3D);
    double DistanceBetweenPointAndLine2D(POINT3D, Line3D);
    double DistanceBetweenPointAndLine2D(PointXYZDouble*, Line3D);
    double DistanceBetweenPoints2D(POINT3D, POINT3D);
    double DistanceBetweenPoints2D(PointXYZDouble*, PointXYZDouble*);
    double DistanceBetweenPointAndLine3DHgt(PointXYZDouble*, Line3D);
    void ComputeGMMfeature(vector<PointXYZDouble*>, vector<GMMPara>&, int);
    void ClusteringByGMM(vector<PointXYZDouble*>, vector<GMMPara>, vector<vector<PointXYZDouble*>>&);
    bool LineExtractionByRansac(vector<PointXYZDouble*>, double, Line3D&);
    bool LineExtractionByRansac(vector<PointXYZDouble*>, double, Line3D&, double&, vector<PointXYZDouble*>&);
    bool LineExtractionWithFixedAngle(vector<PointXYZDouble*>, double, double, Line3D&, vector<PointXYZDouble*>&);
    bool LineExtractionWithFixedAngleForLoopCross(vector<PointXYZDouble*>, double, double, Line3D&, vector<PointXYZDouble*>&);
    void GetMinMaxXY(vector<PointXYZDouble*>, double&, double&, double&, double&);
    void SegmentPoints(vector<PointXYZDouble*>, vector<PointXYZDouble*>&, double);
    void PointGrowing(PointXYZDouble*, vector<PointXYZDouble*>, PtSegment&, int, double);
    double ComputeCloseness(vector<PointXYZDouble*>, vector<Line3D>);
    void SegmentTrackPtsByMDL(int, vector<PointXYZDouble*>, vector<Line3D>&, double);
    void ExtractTrkPts(TrackSegment, double, double, vector<PointXYZDouble*>&,
                      vector<PointXYZDouble*>&, double&, double&);
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
                    "double RailwayExtraction::DistanceBetweenPointAndLine2D(POINT3D",
                    "double RailwayExtraction::DistanceBetweenPointAndLine2D(PointXYZDouble*",
                    "double RailwayExtraction::DistanceBetweenPoints2D(POINT3D",
                    "double RailwayExtraction::DistanceBetweenPoints2D(PointXYZDouble*",
                    "double RailwayExtraction::DistanceBetweenPointAndLine3DHgt(",
                    "void RailwayExtraction::ComputeGMMfeature(",
                    "void RailwayExtraction::ClusteringByGMM(",
                    "bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line)",
                    "void RailwayExtraction::ExtractTrkPts(",
                    "bool RailwayExtraction::LineExtractionByRansac(vector<PointXYZDouble*> pnts, double ThDist, Line3D &_line, double &sigma, vector<PointXYZDouble*> &sptPnts)",
                    "bool RailwayExtraction::LineExtractionWithFixedAngle(",
                    "bool RailwayExtraction::LineExtractionWithFixedAngleForLoopCross(",
                    "void RailwayExtraction::GetMinMaxXY(",
                    "void RailwayExtraction::SegmentPoints(",
                    "void RailwayExtraction::PointGrowing(",
                    "double RailwayExtraction::ComputeCloseness(",
                    "void RailwayExtraction::SegmentTrackPtsByMDL(",
                )
            )
            functions = functions.replace(
                "new Grid[m_NumGrid]", "new Grid[static_cast<size_t>(m_NumGrid)]"
            )
            marker = "\n\tif (MDLScore1 < MDLScore2 && MDLScore1 < MDLScore3)"
            assert functions.count(marker) == 1
            functions = functions.replace(
                marker,
                """
    lastMDLScores = {MDLScore1, MDLScore2, MDLScore3};
    lastMDLCloseness = {closeness1, closeness2, closeness3};
    lastMDLFlags = {flg1, flg2, flg3};
"""
                + marker,
            )
            preamble = """
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace std;
#define PI 3.1415926535897932
struct PointXYZDouble {
    double x, y, z;
    int ClassCode = 0, SegmentID = -1, Intensity = 0;
};
vector<int> random_draws;
size_t random_cursor = 0;
int reference_rand() {
    if (random_cursor >= random_draws.size()) throw runtime_error("random stream exhausted");
    return random_draws[random_cursor++];
}
"""
            path = Path(self.directory.name) / "reference.cpp"
            path.write_text(
                preamble
                + isnan
                + polygon
                + primitives
                + segments
                + higher_z
                + larger
                + declarations
                + "\n#define rand reference_rand\n"
                + functions
                + MAIN
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
double readScalar() {
    string token; cin >> token; return stod(token);
}
void readRandom() {
    size_t count; cin >> count; random_draws.resize(count); random_cursor = 0;
    for (auto &value : random_draws) cin >> value;
}
void printSelection(vector<PointXYZDouble*> points, Cloud &cloud) {
    cout << points.size() << ' ';
    for (auto point : points) cout << point - cloud.points.data() << ' ';
}
vector<PointXYZDouble*> readSelection(Cloud &cloud) {
    size_t count, row; cin >> count;
    vector<PointXYZDouble*> points;
    for (size_t i = 0; i < count; ++i) { cin >> row; points.push_back(&cloud.points.at(row)); }
    return points;
}
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
    } else if (operation == "gmm" || operation == "cluster") {
        size_t count; int classes; cin >> count >> classes;
        Cloud cloud; cloud.points.resize(count);
        vector<PointXYZDouble*> points;
        for (auto &point : cloud.points) {
            point.x = point.y = 0; point.z = readScalar(); points.push_back(&point);
        }
        vector<GMMPara> parameters;
        if (operation == "gmm") engine.ComputeGMMfeature(points, parameters, classes);
        else {
            parameters.resize(classes);
            for (auto &p : parameters) {
                p.mu = readScalar(); p.sigma = readScalar(); p.alpha = readScalar();
            }
        }
        vector<vector<PointXYZDouble*>> clusters;
        engine.ClusteringByGMM(points, parameters, clusters);
        cout << "RESULT ";
        for (auto p : parameters) cout << p.mu << ' ' << p.sigma << ' ' << p.alpha << ' ';
        for (auto group : clusters) printSelection(group, cloud);
    } else if (operation == "ransac") {
        size_t count; double threshold; cin >> count >> threshold;
        Cloud cloud; cloud.points.resize(count); vector<PointXYZDouble*> points;
        for (auto &p : cloud.points) { cin >> p.x >> p.y >> p.z; points.push_back(&p); }
        readRandom(); Line3D line;
        bool ok = engine.LineExtractionByRansac(points, threshold, line);
        cout << "RESULT " << ok << ' ';
        if (ok) { printPoint(line.BeginPt); printPoint(line.EndPt); }
        cout << random_cursor;
    } else if (operation == "heads") {
        size_t firstCount, secondCount; double h1, h2;
        cin >> firstCount >> secondCount >> h1 >> h2;
        Cloud cloud; cloud.points.resize(firstCount + secondCount);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z >> p.ClassCode >> p.Intensity;
        TrackSegment segment;
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            if (i < firstCount) segment.m_Track1.MemPnts.push_back(&cloud.points[i]);
            else segment.m_Track2.MemPnts.push_back(&cloud.points[i]);
        }
        vector<PointXYZDouble*> heads1, heads2;
        size_t count, row;
        cin >> count;
        for (size_t i = 0; i < count; ++i) { cin >> row; heads1.push_back(&cloud.points[row]); }
        cin >> count;
        for (size_t i = 0; i < count; ++i) { cin >> row; heads2.push_back(&cloud.points[row]); }
        readRandom();
        double next1, next2;
        engine.ExtractTrkPts(segment, h1, h2, heads1, heads2, next1, next2);
        cout << "RESULT " << next1 << ' ' << next2 << ' ';
        printSelection(heads1, cloud); printSelection(heads2, cloud);
        for (auto &p : cloud.points) cout << p.ClassCode << ' ';
        cout << random_cursor;
    } else if (operation == "ransac500") {
        size_t count; double threshold; cin >> count >> threshold;
        Cloud cloud; cloud.points.resize(count); vector<PointXYZDouble*> points;
        for (auto &p : cloud.points) { cin >> p.x >> p.y >> p.z; points.push_back(&p); }
        readRandom(); Line3D line; double sigma = -1; vector<PointXYZDouble*> support;
        bool ok = engine.LineExtractionByRansac(points, threshold, line, sigma, support);
        cout << "RESULT " << ok << ' ';
        if (ok) { printPoint(line.BeginPt); printPoint(line.EndPt); cout << sigma << ' '; }
        printSelection(support, cloud); cout << random_cursor;
    } else if (operation == "fixed" || operation == "loop_fixed") {
        size_t count; double angle, threshold;
        cin >> count >> angle >> threshold >> engine.m_skip_rate;
        Cloud cloud; cloud.points.resize(count);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z;
        auto points = readSelection(cloud);
        Line3D line; vector<PointXYZDouble*> support;
        bool initialized; cin >> initialized;
        if (initialized) {
            line.BeginPt = readPoint(); line.EndPt = readPoint();
            line.GetLineParameter(); line.GetAngle();
            support = readSelection(cloud); line.MemPts = support;
        }
        readRandom();
        bool ok = operation == "fixed"
            ? engine.LineExtractionWithFixedAngle(points, angle, threshold, line, support)
            : engine.LineExtractionWithFixedAngleForLoopCross(points, angle, threshold, line, support);
        cout << "RESULT " << ok << ' ';
        if (ok) { printPoint(line.BeginPt); printPoint(line.EndPt); }
        printSelection(support, cloud);
        if (ok) printSelection(line.MemPts, cloud);
        cout << random_cursor;
    } else if (operation == "group") {
        size_t count; double threshold; cin >> count >> threshold;
        Cloud cloud; cloud.points.resize(count);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z >> p.ClassCode >> p.SegmentID;
        auto points = readSelection(cloud);
        vector<PointXYZDouble*> selected;
        engine.SegmentPoints(points, selected, threshold);
        cout << "RESULT "; printSelection(selected, cloud);
        for (auto &p : cloud.points) cout << p.SegmentID << ' ';
        for (auto &p : cloud.points) cout << p.ClassCode << ' ';
    } else if (operation == "mdl") {
        size_t count; int mode; double angle;
        cin >> count >> mode >> angle >> engine.m_ThRansac >> engine.m_ThDistGrouping;
        cin >> engine.m_wMDL >> engine.m_skip_rate;
        Cloud cloud; cloud.points.resize(count);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z;
        auto points = readSelection(cloud);
        readRandom(); vector<Line3D> lines;
        engine.SegmentTrackPtsByMDL(mode, points, lines, angle);
        cout << "RESULT " << lines.size() << ' ';
        for (auto line : lines) {
            printPoint(line.BeginPt); printPoint(line.EndPt); printSelection(line.MemPts, cloud);
        }
        for (double score : engine.lastMDLScores) cout << score << ' ';
        for (double value : engine.lastMDLCloseness) cout << value << ' ';
        for (int flag : engine.lastMDLFlags) cout << flag << ' ';
        cout << random_cursor << ' ';
        for (auto &p : cloud.points) cout << p.SegmentID << ' ';
    } else return 2;
    cout << '\n';
    return 0;
}
"""
