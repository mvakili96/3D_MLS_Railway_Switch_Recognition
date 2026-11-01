"""Compile original source extracts, not a second Python-derived implementation.

PCL/Windows declarations are replaced by small data shims. An allocation-size cast
accommodates the source's double array length in GCC. For stochastic comparisons,
rand() reads a supplied raw-draw stream instead of the host CRT generator. Tested
arithmetic, branches and thresholds remain unchanged. MDL diagnostic assignments
capture its existing local scores/flags without changing the selection rules.
Gauge diagnostic assignments likewise only copy existing intermediate values.
observe_prefix stops the original selection body before its discarded local gauge
adjustment; it can check defined point vectors when a tied three-line comparison
would subsequently read an uninitialized local line in the full native function.
The bundled matrix/vector templates are included for Kalman tests. Input-only
references and size getters receive mechanical const qualifiers for GCC rvalue
binding; matrix arithmetic and LU branches remain original. Inverse diagnostics
capture its valid flag and count the source's existing TINY substitutions.
The tracking oracle compiles the original loop through DuplicatePolyPair, with
an explicit accepted-step test limit. The omitted tail is print/progress/render
callbacks and the separate recognition postpass. Stop diagnostics copy branches;
predicted-window diagnostics copy existing lengths/counts without changing math.
Recognition uses the original BinaryString/TrackStack structs, catalogue and
matching functions. Its batch wrapper extracts only the actual signature/scoring
postpass, stopping before toe detection; score diagnostics only copy locals.
"""

from pathlib import Path
import re
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


def matrix_source() -> str:
    def methods(name: str, owner: str, names: tuple[str, ...]) -> str:
        original = (SOURCE / "etc" / name).read_text()
        blocks = re.split(r"(?=template <typename _T>)", original)
        selected = []
        for block in blocks:
            signature = "".join(block.split("{", 1)[0].split())
            if not any("::" + method + "(" in signature for method in names):
                continue
            if (
                owner == "Matrix"
                and "::operator*(" in signature
                and "constMatrix<_T>&" not in signature
            ):
                continue
            if "::operator()(" in signature and "_Col" not in signature:
                continue
            selected.append(block)
        return "\n".join(selected)

    # Unused legacy methods contain unrelated compile errors; extract only the
    # actual matrix/vector operations reached by the two Kalman routines.
    parts = [
        (SOURCE / "etc" / "vector.h").read_text(),
        methods("vector.inl", "Vector", ("Vector", "~Vector", "Define", "__Create", "__Delete")),
        (SOURCE / "etc" / "matrix.h").read_text(),
        methods(
            "matrix.inl",
            "Matrix",
            (
                "Matrix",
                "~Matrix",
                "Define",
                "__Create",
                "__Delete",
                "operator+",
                "operator-",
                "operator*",
                "operator=",
                "operator[]",
                "operator()",
                "Inverse",
                "Transpose",
                "LUbacksb",
                "LUDecompose",
            ),
        ),
    ]
    source = "\n".join(
        line
        for part in parts
        for line in part.splitlines()
        if not line.startswith(("#include", "#pragma"))
    )
    source = source.replace("IN Matrix<_T>&", "IN const Matrix<_T>&")
    source = source.replace("IN Vector<_T>&", "IN const Vector<_T>&")
    for getter in ("GetRows", "GetCols", "GetSize"):
        source = source.replace(getter + "() {", getter + "() const {")
    inverse = function(source, "Matrix<_T> Matrix<_T>::Inverse()")
    modified = inverse.replace("{", "{\nlast_inverse_valid=false; matrix_zero_pivots=0;", 1)
    modified = modified.replace(
        "ret.m_isValid = true;", "ret.m_isValid = true; last_inverse_valid=true;"
    )
    source = source.replace(inverse, modified)
    source = source.replace("_LUd[i][i] = TINY;", "{ _LUd[i][i] = TINY; ++matrix_zero_pivots; }")
    return (
        "\n#define IN\n#define OUT\nusing UINT = unsigned int;\nbool last_inverse_valid;\nint matrix_zero_pivots;\n"
        + source
    )


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
struct TrackLayer {
    vector<TrackSegment> m_TrackSegList;
    vector<POINT3D> m_RailVec1, m_RailVec2, m_RailVecCenter;
    vector<Line3D> m_updateBnd, m_updateBnd_turnout;
    vector<ToePt> m_ToePts;
};
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
    double lastGaugeAngle, lastGaugeGap, lastGaugeScale, lastWeightedDistance;
    vector<double> lastGaugeErrors, lastGaugeDistances;
    Grid *m_pGrid;
    bool m_bInit = false;
    double m_distFromSp = -1, m_InitTrkHgt1, m_InitTrkHgt2;
    TrackLayer *currentLayer;
    InitialLocation initialLocation;
    int m_ThMinPts, m_ThMinTrkPts, referenceMaxSteps;
    int lastTrackingStop = 0;
    vector<TrackStack> m_TrkStks;
    TrackStack referenceActiveStack;
    vector<int> m_LoopCross;
    vector<vector<double>> trackingWindows;
    vector<BinaryString> m_Tmplts, m_TmpltBegin, m_TmpltEnd;
    vector<double> lastRecognitionUnary, lastRecognitionContext, lastRecognitionScores;
    void DefineTemplates();
    void RecognitionTurnoutType(vector<BinaryString>, BinaryString, BinaryString&);
    void TemplateMatching(vector<BinaryString>, BinaryString&);
    void PrintTurnoutType(int);
    void RecognizeStackBatch(bool);
    bool DetectToePnts(int, vector<TrackSegment>, ToePt&);
    void DetectStackToes(bool);
    vector<ToePt> m_ToePts;
    vector<PointXYZDouble*> lastToeCandidates, lastToeSegment, lastToeSupport;
    Line3D lastToeLine;
    POINT3D lastToeSelected;
    double lastToeSigma;
    int lastToeStage = 0;
    void progressCallback(double, double) {}
    void generateGrid();
    bool Setinitiallocation(int, POINT3D, double);
    void TrackingLoop(int, bool);
    void GenerateTrkSegAssignMemPts(TrackSegment&, TrackSegment, Matrix<double>, Matrix<double>, double, double, int);
    void GetLnsFromVtx(vector<POINT3D>, vector<Line3D>&);
    int ReCount(int, int);
    void KalmanPredict(int, double, double, Matrix<double>, Matrix<double>, Matrix<double>&, Matrix<double>&);
    void KalmanUpdate(Matrix<double>, Matrix<double>, Matrix<double>, double, Matrix<double>&, Matrix<double>&);
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
    int GetDirectionMode(double, double, double, double);
    void AdjustTwoLines(Line3D&, Line3D&, int, int, int, double);
    void AdjustTwoLinesWithObservation(POINT3D, POINT3D, Line3D&, Line3D&,
                                      vector<PointXYZDouble*>, vector<PointXYZDouble*>, int, int, int, double);
    void DetermineObservation(int, TrackStack, Line3D, Line3D, vector<Line3D>, vector<Line3D>, int,
                              vector<PointXYZDouble*>&, vector<PointXYZDouble*>&,
                              vector<PointXYZDouble*>&, vector<PointXYZDouble*>&,
                              vector<PointXYZDouble*>&, vector<PointXYZDouble*>&);
    void DetermineObservationPrefix(int, TrackStack, Line3D, Line3D, vector<Line3D>, vector<Line3D>, int,
                                    vector<PointXYZDouble*>&, vector<PointXYZDouble*>&,
                                    vector<PointXYZDouble*>&, vector<PointXYZDouble*>&,
                                    vector<PointXYZDouble*>&, vector<PointXYZDouble*>&);
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
                    "int RailwayExtraction::GetDirectionMode(",
                    "void RailwayExtraction::AdjustTwoLines(",
                    "void RailwayExtraction::AdjustTwoLinesWithObservation(",
                    "void RailwayExtraction::DetermineObservation(",
                    "bool RailwayExtraction::Setinitiallocation(",
                    "void RailwayExtraction::GetLnsFromVtx(",
                    "int RailwayExtraction::ReCount(",
                    "void RailwayExtraction::KalmanPredict(",
                    "void RailwayExtraction::KalmanUpdate(",
                    "void RailwayExtraction::GenerateTrkSegAssignMemPts(",
                    "void RailwayExtraction::DefineTemplates(",
                    "void RailwayExtraction::RecognitionTurnoutType(",
                    "void RailwayExtraction::TemplateMatching(",
                    "void RailwayExtraction::PrintTurnoutType(",
                    "bool RailwayExtraction::DetectToePnts(",
                )
            )
            toe = function(extraction, "bool RailwayExtraction::DetectToePnts(")
            modified = toe.replace(
                "ln.SetEndingPnts(minX, maxX, minY, maxY);",
                "ln.SetEndingPnts(minX, maxX, minY, maxY);\nlastToeCandidates=tmpPnts; lastToeSegment=segmentPts; lastToeSupport=sptPnts; lastToeLine=ln; lastToeSigma=sigma; lastToeStage=1;",
            )
            for index in ("0", "end"):
                marker = f"\n\t\tln_track1 = trkSeg[{index}].m_Track1.MemLn;"
                modified = modified.replace(
                    marker, "\nlastToeSelected=tmpPt; lastToeStage=2;" + marker
                )
            functions = functions.replace(toe, modified)
            toe_start = extraction.index(
                "\n\t\t//detect Toes",
                extraction.index("\nvoid RailwayExtraction::Trackingrailway("),
            )
            toe_end = extraction.index("\n\t\t//Detect switch box", toe_start)
            functions += (
                "\nvoid RailwayExtraction::DetectStackToes(bool mainTrkflg) { if (mainTrkflg) {\n"
                + extraction[toe_start:toe_end]
                + "\n} }"
            )
            recognition = function(extraction, "void RailwayExtraction::RecognitionTurnoutType(")
            modified = recognition.replace(
                "\n\tdouble maxScore = 0;",
                "\nlastRecognitionUnary=UnaryScore; lastRecognitionContext=ContextScore; lastRecognitionScores=Score;\n\tdouble maxScore = 0;",
            )
            functions = functions.replace(recognition, modified)
            matching = function(extraction, "void RailwayExtraction::TemplateMatching(")
            modified = matching.replace(
                "\n\tint maxScore = 0;",
                "\nlastRecognitionUnary.assign(score.begin(),score.end()); lastRecognitionContext.clear(); lastRecognitionScores=lastRecognitionUnary;\n\tint maxScore = 0;",
            )
            functions = functions.replace(matching, modified)
            start = extraction.index(
                "\n\tDefineTemplates();",
                extraction.index("\nvoid RailwayExtraction::Trackingrailway("),
            )
            end = extraction.index("\n\t\t//validation of template matching", start)
            functions += (
                "\nvoid RailwayExtraction::RecognizeStackBatch(bool mainTrkflg) {\n"
                + extraction[start:end]
                + "\n}\n}"
            )
            # This routine has an unmatched brace inside a commented-out loop;
            # select its boundary directly instead of counting comment braces.
            tracking = extraction[
                extraction.index("\nvoid RailwayExtraction::Trackingrailway(") + 1 :
            ]
            tracking = tracking[
                : tracking.index("\n\t\tPOINT3D nPt = nextSeg.m_Track1.MemLn.EndPt;")
            ]
            tracking += "\nif (iter >= referenceMaxSteps) { lastTrackingStop=5; break; }\n}\nreferenceActiveStack=trkStk;\n}"
            tracking = tracking.replace("::Trackingrailway(", "::TrackingLoop(")
            for marker, code in (
                ("# of points in Track Window is too small!!", 1),
                ("# of head points in Window", 2),
                ("Track Guage difference:", 3),
                ("Track End by trajectory", 4),
            ):
                position = tracking.index('printf("' + marker)
                tracking = tracking[:position] + f"lastTrackingStop={code}; " + tracking[position:]
            functions += "\n" + tracking
            generation = function(extraction, "void RailwayExtraction::GenerateTrkSegAssignMemPts(")
            diagnostic = "\ntrackingWindows.push_back({Length, double(nextSeg.m_Track1.MemPnts.size()), double(nextSeg.m_Track2.MemPnts.size())});\n"
            functions = functions.replace(generation, generation[:-1] + diagnostic + "}")
            observation = function(extraction, "void RailwayExtraction::DetermineObservation(")
            prefix = observation[: observation.index("\n\tint mode1, mode2;")] + "\n}"
            functions += "\n" + prefix.replace(
                "::DetermineObservation(", "::DetermineObservationPrefix("
            )
            functions = functions.replace(
                "tmpln = tmpln.ExtendEndingPts(_dist / fdist);",
                """lastGaugeAngle = ang; lastGaugeGap = fdist; lastGaugeScale = _dist / fdist;
    lastWeightedDistance = nan(\"\"); lastGaugeErrors.clear(); lastGaugeDistances.clear();
    tmpln = tmpln.ExtendEndingPts(_dist / fdist);""",
            )
            functions = functions.replace(
                "weightedDist = weightedDist / weight;",
                """weightedDist = weightedDist / weight;
    lastGaugeAngle = ang; lastGaugeGap = DistanceBetweenPoints2D(pt1, pt2);
    lastGaugeScale = _dist / weightedDist; lastWeightedDistance = weightedDist;
    lastGaugeErrors.clear(); lastGaugeDistances.clear();
    for (auto value : DiffList) { lastGaugeErrors.push_back(value.z); lastGaugeDistances.push_back(value.x); }
""",
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
                + matrix_source()
                + isnan
                + polygon
                + primitives
                + header[header.index("struct BinaryString {") : header.index("struct ToePt {")]
                + header[header.index("struct ToePt {") : header.index("struct LoopCross {")]
                + segments
                + higher_z
                + larger
                + header[
                    header.index("struct InitialLocation {") : header.index("struct GradeSect {")
                ]
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
Line3D readStateLine(Cloud &cloud) {
    Line3D line;
    line.BeginPt = readPoint(); line.EndPt = readPoint(); line.CenterPt = readPoint();
    line.Ang = readScalar(); line.A2D = readScalar(); line.B2D = readScalar(); line.C2D = readScalar();
    line.MemPts = readSelection(cloud);
    return line;
}
Matrix<double> readMatrix(int rows, int columns) {
    Matrix<double> result(rows, columns, 0);
    for (int row = 0; row < rows; ++row)
        for (int column = 0; column < columns; ++column) result(row, column) = readScalar();
    return result;
}
void printMatrix(Matrix<double> value) {
    for (UINT row = 0; row < value.GetRows(); ++row)
        for (UINT column = 0; column < value.GetCols(); ++column) cout << value(row, column) << ' ';
}
void printStateLine(Line3D line, Cloud &cloud) {
    printPoint(line.BeginPt); printPoint(line.EndPt); printPoint(line.CenterPt);
    cout << line.Ang << ' ' << line.A2D << ' ' << line.B2D << ' ' << line.C2D << ' ';
    printSelection(line.MemPts, cloud);
}
void printTrackingSegment(TrackSegment segment, Cloud &cloud, bool fitted) {
    printPoint(segment.m_Center); cout << segment.m_D << ' ';
    if (fitted) cout << segment.distFromSp << ' ';
    for (auto window : {segment.m_Track1, segment.m_Track2}) {
        printPoint(window.CenterPt); cout << window.ori << ' '; printVertices(window.vtx);
        for (auto points : {window.MemPnts, window.slMemPnts, window.srMemPnts}) printSelection(points, cloud);
        cout << window.bTurnOut << ' ' << window.bLeftTag << ' ' << window.bRightTag << ' ';
        if (fitted) printStateLine(window.MemLn, cloud);
    }
}
BinaryString readBinary() {
    BinaryString value; cin >> value.Type; value.Score = readScalar();
    cin >> value.LabelPosTurnout >> value.LabelDirection >> value.LabelTurnoutType >> value.LabelPosToe;
    size_t count; cin >> count;
    for (size_t i=0; i<count; ++i) { int bit; cin >> bit; value.bStrings.push_back(bit); }
    return value;
}
void printBinary(BinaryString value) {
    cout << value.Type << ' ' << value.Score << ' ' << value.LabelPosTurnout << ' '
         << value.LabelDirection << ' ' << value.LabelTurnoutType << ' ' << value.LabelPosToe << ' '
         << value.bStrings.size() << ' ';
    for (auto bit : value.bStrings) cout << bit << ' ';
}
ToePt readToe(bool &hasGeometry) {
    ToePt value; cin >> value.TrkType >> value.used >> hasGeometry;
    if (hasGeometry) {
        value.CenterPt=readPoint(); value.LeftPt=readPoint(); value.RightPt=readPoint();
        value.TrkAng=readScalar(); cin >> value.TrkMode;
    }
    return value;
}
void printToe(ToePt value, bool hasGeometry) {
    cout << value.TrkType << ' ' << value.used << ' ' << hasGeometry << ' ';
    if (hasGeometry) {
        printPoint(value.CenterPt); printPoint(value.LeftPt); printPoint(value.RightPt);
        cout << value.TrkAng << ' ' << value.TrkMode << ' ';
    }
}
vector<TrackSegment> readToeSegments(Cloud &cloud) {
    size_t count; cin >> count; vector<TrackSegment> segments(count);
    for (auto &segment : segments) {
        segment.m_Center=POINT3D(0,0,0); segment.distFromSp=0;
        for (auto window : {&segment.m_Track1, &segment.m_Track2}) {
            window->CenterPt=POINT3D(0,0,0); window->ori=0;
            bool hasLine; cin >> hasLine;
            if (hasLine) window->MemLn=readStateLine(cloud);
            window->slMemPnts=readSelection(cloud); window->srMemPnts=readSelection(cloud);
        }
    }
    return segments;
}
int main() {
    cout << setprecision(17);
    string operation;
    cin >> operation;
    RailwayExtraction engine;
    if (operation == "toe" || operation == "toe_batch") {
        cin >> engine.m_ThRansac; size_t count; cin >> count;
        Cloud cloud; cloud.points.resize(count); engine.point_cloud=&cloud;
        for (auto &point : cloud.points) cin >> point.x >> point.y >> point.z >> point.ClassCode >> point.SegmentID;
        TrackLayer layer; engine.currentLayer=&layer;
        if (operation == "toe") {
            int type; cin >> type; auto segments=readToeSegments(cloud);
            bool initialized; auto output=readToe(initialized); readRandom();
            bool ok=engine.DetectToePnts(type,segments,output);
            cout << "RESULT " << ok << ' '; printToe(output, initialized || ok);
            cout << engine.lastToeStage << ' ';
            if (engine.lastToeStage > 0) {
                for (auto points : {engine.lastToeCandidates,engine.lastToeSegment,engine.lastToeSupport}) printSelection(points,cloud);
                cout << engine.lastToeSigma << ' '; printStateLine(engine.lastToeLine,cloud);
                printPoint(engine.lastToeSelected);
            }
        } else {
            bool mainTrack; cin >> mainTrack >> count;
            for (size_t i=0; i<count; ++i) {
                TrackStack stack; stack.trkBinary=readBinary(); stack.trkSegs=readToeSegments(cloud);
                engine.m_TrkStks.push_back(stack);
            }
            cin >> count;
            for (size_t i=0; i<count; ++i) { bool initialized; engine.m_ToePts.push_back(readToe(initialized)); }
            readRandom(); engine.DetectStackToes(mainTrack);
            cout << "RESULT " << engine.m_ToePts.size() << ' ';
            for (auto toe : engine.m_ToePts) printToe(toe,true);
            cout << layer.m_ToePts.size() << ' '; for (auto toe : layer.m_ToePts) printToe(toe,true);
        }
        for (auto point : cloud.points) cout << point.ClassCode << ' ' << point.SegmentID << ' ';
        cout << random_cursor << ' ';
    } else if (operation == "templates") {
        engine.DefineTemplates(); cout << "RESULT ";
        for (auto catalogue : {engine.m_Tmplts, engine.m_TmpltBegin, engine.m_TmpltEnd}) {
            cout << catalogue.size() << ' '; for (auto value : catalogue) printBinary(value);
        }
    } else if (operation == "binary_labels") {
        size_t count; cin >> count; cout << "RESULT ";
        for (size_t i=0; i<count; ++i) { int type; cin >> type; BinaryString value; value.SetLables(type); printBinary(value); }
    } else if (operation == "recognize" || operation == "match_raw") {
        auto observed=readBinary(), previous=readBinary();
        bool custom; cin >> custom; vector<BinaryString> templates;
        if (custom) {
            size_t count; cin >> count;
            for (size_t i=0; i<count; ++i) templates.push_back(readBinary());
        } else { engine.DefineTemplates(); templates=engine.m_Tmplts; }
        if (operation == "recognize") engine.RecognitionTurnoutType(templates, previous, observed);
        else engine.TemplateMatching(templates, observed);
        cout << "RESULT "; printBinary(observed);
        for (auto scores : {engine.lastRecognitionUnary, engine.lastRecognitionContext, engine.lastRecognitionScores}) {
            cout << scores.size() << ' '; for (auto score : scores) cout << score << ' ';
        }
    } else if (operation == "stack_recognition") {
        size_t count; bool mainTrack; cin >> mainTrack >> count;
        for (size_t i=0; i<count; ++i) {
            TrackStack stack; stack.trkBinary=readBinary(); size_t length; cin >> length;
            stack.trkSegs.resize(length);
            for (auto &segment : stack.trkSegs)
                cin >> segment.m_Track2.bLeftTag >> segment.m_Track2.bRightTag
                    >> segment.m_Track1.bLeftTag >> segment.m_Track1.bRightTag;
            engine.m_TrkStks.push_back(stack);
        }
        engine.RecognizeStackBatch(mainTrack);
        cout << "RESULT " << engine.m_TrkStks.size() << ' ';
        for (auto stack : engine.m_TrkStks) printBinary(stack.trkBinary);
    } else if (operation == "track") {
        int direction; bool mainTrack;
        cin >> engine.m_GridSize >> engine.m_TrkGuage >> engine.m_gLength >> engine.m_gWidth
            >> engine.m_ThRansac >> engine.m_ThMinPts >> engine.m_ThMinTrkPts >> engine.m_skip_rate
            >> engine.m_wMDL >> engine.m_ThDistGrouping >> direction >> mainTrack
            >> engine.m_distFromSp >> engine.referenceMaxSteps >> engine.m_InitTrkHgt1 >> engine.m_InitTrkHgt2;
        engine.m_BoundMin = readPoint(); engine.m_BoundMax = readPoint();
        size_t count; cin >> count;
        Cloud cloud; cloud.points.resize(count); engine.point_cloud = &cloud;
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z >> p.ClassCode >> p.SegmentID;
        TrackSegment segment; cin >> segment.distFromSp;
        for (auto window : {&segment.m_Track1, &segment.m_Track2}) {
            window->CenterPt = readPoint(); cin >> window->ori;
            for (int i = 0; i < 4; ++i) window->vtx.push_back(readPoint());
            window->MemPnts = readSelection(cloud); window->MemLn = readStateLine(cloud);
        }
        segment.GetCenterOfTwoTracks(); segment.GetDistanceBetweenTwoCenter();
        TrackLayer layer; layer.m_TrackSegList.push_back(segment); engine.currentLayer = &layer;
        layer.m_RailVec1.push_back(segment.m_Track1.CenterPt);
        layer.m_RailVec2.push_back(segment.m_Track2.CenterPt);
        layer.m_RailVecCenter.push_back(segment.m_Center);
        for (auto window : {segment.m_Track1, segment.m_Track2}) {
            vector<Line3D> edges; engine.GetLnsFromVtx(window.vtx, edges);
            layer.m_updateBnd.insert(layer.m_updateBnd.end(), edges.begin(), edges.end());
        }
        engine.generateGrid(); readRandom(); engine.m_bInit = true;
        if (engine.referenceMaxSteps > 0) engine.TrackingLoop(direction, mainTrack);
        cout << "\nRESULT " << engine.lastTrackingStop << ' ' << engine.m_distFromSp << ' ';
        cout << layer.m_TrackSegList.size() << ' ';
        for (auto item : layer.m_TrackSegList) printTrackingSegment(item, cloud, true);
        for (auto points : {layer.m_RailVec1, layer.m_RailVec2, layer.m_RailVecCenter}) {
            cout << points.size() << ' '; printVertices(points);
        }
        for (auto edges : {layer.m_updateBnd, layer.m_updateBnd_turnout}) {
            cout << edges.size() << ' ';
            for (auto edge : edges) { printPoint(edge.BeginPt); printPoint(edge.EndPt); }
        }
        cout << engine.m_TrkStks.size() << ' ';
        for (auto stack : engine.m_TrkStks) {
            cout << stack.trkSegs.size() << ' ';
            for (auto item : stack.trkSegs) printTrackingSegment(item, cloud, false);
        }
        cout << engine.referenceActiveStack.trkSegs.size() << ' ';
        for (auto item : engine.referenceActiveStack.trkSegs) printTrackingSegment(item, cloud, false);
        cout << engine.trackingWindows.size() << ' ';
        for (auto attempt : engine.trackingWindows) for (auto value : attempt) cout << value << ' ';
        for (auto p : cloud.points) cout << p.ClassCode << ' ' << p.SegmentID << ' ';
        cout << random_cursor << ' ';
        delete[] engine.m_pGrid;
    } else if (operation == "predict") {
        int mode; double angle, length; cin >> mode >> angle >> length;
        auto point = readMatrix(3, 1), covariance = readMatrix(3, 3);
        Matrix<double> nextPoint(3, 1, 0), nextCovariance(3, 3, 0);
        engine.KalmanPredict(mode, angle, length, point, covariance, nextPoint, nextCovariance);
        cout << "RESULT "; printMatrix(nextPoint); printMatrix(nextCovariance);
    } else if (operation == "update") {
        double sigma = readScalar();
        auto point = readMatrix(3, 1), covariance = readMatrix(3, 3), observation = readMatrix(3, 1);
        Matrix<double> nextPoint(3, 1, 0), nextCovariance(3, 3, 0);
        engine.KalmanUpdate(point, covariance, observation, sigma, nextPoint, nextCovariance);
        cout << "RESULT "; printMatrix(nextPoint); printMatrix(nextCovariance);
        cout << last_inverse_valid << ' ' << matrix_zero_pivots << ' ';
    } else if (operation == "initialize") {
        int mode; double angle; cin >> mode >> angle; auto point = readPoint();
        cin >> engine.m_TrkGuage >> engine.m_gLength >> engine.m_gWidth >> engine.m_GridSize;
        engine.m_BoundMin = readPoint(); engine.m_BoundMax = readPoint();
        size_t count; cin >> count;
        Cloud cloud; cloud.points.resize(count); engine.point_cloud = &cloud;
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z >> p.ClassCode >> p.SegmentID;
        engine.generateGrid(); readRandom();
        TrackLayer layer; engine.currentLayer = &layer;
        bool ok = engine.Setinitiallocation(mode, point, angle);
        cout << "\nRESULT " << ok << ' ' << engine.m_bInit << ' ' << engine.m_distFromSp << ' ';
        cout << layer.m_TrackSegList.size() << ' ';
        if (ok) {
            cout << engine.m_InitTrkHgt1 << ' ' << engine.m_InitTrkHgt2 << ' ';
            auto location = engine.initialLocation;
            cout << location.x << ' ' << location.y << ' ' << location.z << ' '
                 << location.second_x << ' ' << location.second_y << ' ' << location.second_z << ' '
                 << location.angle << ' ' << location.mode << ' ';
            auto segment = layer.m_TrackSegList.back();
            printPoint(segment.m_Center); cout << segment.m_D << ' ' << segment.distFromSp << ' ';
            for (auto window : {segment.m_Track1, segment.m_Track2}) {
                printPoint(window.CenterPt); cout << window.ori << ' '; printVertices(window.vtx);
                printSelection(window.MemPnts, cloud); printStateLine(window.MemLn, cloud);
            }
        }
        for (auto points : {layer.m_RailVec1, layer.m_RailVec2, layer.m_RailVecCenter}) {
            cout << points.size() << ' '; printVertices(points);
        }
        cout << layer.m_updateBnd.size() << ' ';
        for (auto line : layer.m_updateBnd) { printPoint(line.BeginPt); printPoint(line.EndPt); }
        for (auto p : cloud.points) cout << p.ClassCode << ' ' << p.SegmentID << ' ';
        cout << random_cursor << ' ';
        delete[] engine.m_pGrid;
    } else if (operation == "gauge_basic" || operation == "gauge_weighted") {
        size_t count; double target; int m1, m2, mode;
        cin >> count >> engine.m_gLength >> target >> m1 >> m2 >> mode;
        Cloud cloud; cloud.points.resize(count);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z;
        auto first = readStateLine(cloud), second = readStateLine(cloud);
        if (operation == "gauge_basic") engine.AdjustTwoLines(first, second, m1, m2, mode, target);
        else {
            auto ref1 = readPoint(), ref2 = readPoint();
            auto support1 = readSelection(cloud), support2 = readSelection(cloud);
            engine.AdjustTwoLinesWithObservation(ref1, ref2, first, second, support1, support2,
                                                m1, m2, mode, target);
        }
        cout << "RESULT "; printStateLine(first, cloud); printStateLine(second, cloud);
        cout << engine.lastGaugeAngle << ' ' << engine.lastGaugeGap << ' '
             << engine.lastGaugeScale << ' ' << engine.lastWeightedDistance << ' ';
        cout << engine.lastGaugeErrors.size() << ' ';
        for (auto x : engine.lastGaugeErrors) cout << x << ' ';
        cout << engine.lastGaugeDistances.size() << ' ';
        for (auto x : engine.lastGaugeDistances) cout << x << ' ';
    } else if (operation == "observe" || operation == "observe_prefix") {
        size_t count, size1, size2, stackSize; int trackMode, lineMode;
        cin >> count >> engine.m_TrkGuage >> engine.m_gLength >> trackMode >> stackSize >> lineMode;
        Cloud cloud; cloud.points.resize(count);
        for (auto &p : cloud.points) cin >> p.x >> p.y >> p.z;
        auto guide1 = readStateLine(cloud), guide2 = readStateLine(cloud);
        vector<Line3D> lines1, lines2;
        cin >> size1; for (size_t i = 0; i < size1; ++i) lines1.push_back(readStateLine(cloud));
        cin >> size2; for (size_t i = 0; i < size2; ++i) lines2.push_back(readStateLine(cloud));
        TrackStack stack; stack.trkSegs.resize(stackSize);
        vector<PointXYZDouble*> obs1, obs2, left1, right1, left2, right2;
        if (operation == "observe")
            engine.DetermineObservation(trackMode, stack, guide1, guide2, lines1, lines2, lineMode,
                                        obs1, obs2, left1, right1, left2, right2);
        else engine.DetermineObservationPrefix(trackMode, stack, guide1, guide2, lines1, lines2, lineMode,
                                               obs1, obs2, left1, right1, left2, right2);
        cout << "RESULT ";
        for (auto points : {obs1, obs2, left1, right1, left2, right2}) printSelection(points, cloud);
    } else if (operation == "window") {
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
