"""Compile actual 2018 evaluator bodies; no Python-derived scoring oracle.

Only MFC presentation and data declarations are shimmed. Instrumentation copies
existing summary locals before the original printf calls; arithmetic is intact.
ClassCodeRe is never read for an unassigned native point (it is uninitialized).
This comparison does not execute the Windows application or its LAS SDK.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from railway_mls.evaluation.audit import reference_manifest
from railway_mls.evaluation.io import file_fingerprint


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "evaluation data/RailwayExtractor_20180306_evaluation/Source/Viewer"


def extract(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 0
    for i in range(opening, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i+1]
    raise ValueError(signature)


class CppEvaluationReference:
    def __init__(self):
        if not shutil.which("g++") or not (SOURCE / "ViewerDoc.cpp").is_file():
            raise unittest.SkipTest("2018 evaluator source and g++ required")
        for entry in reference_manifest()["files"]:
            if Path(entry["path"]).name in ("ViewerDoc.cpp", "LIDARPT.h"):
                if file_fingerprint(ROOT / entry["path"])["sha256"] != entry["sha256"]:
                    raise AssertionError("2018 evaluation reference source drift")
        header = (SOURCE / "las/LIDARPT.h").read_text(encoding="latin1")
        doc = (SOURCE / "ViewerDoc.cpp").read_text(encoding="latin1")
        line = "struct Line3D { POINT3D BeginPt,EndPt; double A2D,B2D,C2D; vector<LiDPOINT*> TrkPts,nonTrkPts;\n"
        line += extract(header, "void GetLineParameter(){") + "\n"
        line += extract(header, "vector<POINT3D> GetBufferVtxOfLine(double buffersize)") + "\n};\n"
        declarations = r'''
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
using namespace std;
struct POINT3D { double x,y,z; POINT3D(){} POINT3D(double a,double b,double c):x(a),y(b),z(c){} };
struct LiDPOINT { double x,y,z; int ClassCode,SegmentID=-1,ClassCodeRe; };
struct Stats {double mh,sh,mv,sv,precision,recall; int tp,fp,tn,fn; } stats;
'''
        klass = r'''
class CViewerDoc { public:
vector<LiDPOINT> m_LiDAR; vector<Line3D> m_RailVec1,m_RailVec2;
int m_ClassView;
void UpdateAllViews(void*){}
POINT3D ComputeFootOfPerpendicular(POINT3D p, Line3D ln);
double DistanceBetweenPoints2D(POINT3D p1,POINT3D p2);
double DistanceBetweenPointAndLine2D(LiDPOINT* p,Line3D l);
double DistanceOfHeightBetweenPointAndLine3D(LiDPOINT* p,Line3D ln);
void GetPntsInsidePolygon(vector<POINT3D> inVtx,vector<LiDPOINT*> inPts,vector<LiDPOINT*> &outPts);
void OnHeadptsandtrackEvaluation();
};
'''
        helpers = "\n".join(extract(doc, signature) for signature in (
            "POINT3D CViewerDoc::ComputeFootOfPerpendicular(",
            "double CViewerDoc::DistanceBetweenPoints2D(POINT3D",
            "double CViewerDoc::DistanceBetweenPointAndLine2D(LiDPOINT*",
            "double CViewerDoc::DistanceOfHeightBetweenPointAndLine3D(",
            "void CViewerDoc::GetPntsInsidePolygon(",
        ))
        evaluation = extract(doc, "void CViewerDoc::OnHeadptsandtrackEvaluation()")
        marker = 'printf("Average Distance:'
        capture = '''stats = {aveDist,stdDist,aveHgt,stdHgt,
            (double)nTP/((double)nTP+(double)nFP)*100,
            (double)nTP/((double)nTP+(double)nFN)*100,nTP,nFP,nTN,nFN};\n'''
        evaluation = evaluation.replace(marker, capture+marker, 1)
        main = r'''
int main(){
    int nl,nr,np; cin >> nl >> nr >> np; CViewerDoc doc;
    for(int side=0;side<2;side++) for(int i=0;i<(side?nr:nl);i++){
        Line3D l; cin >> l.BeginPt.x >> l.BeginPt.y >> l.BeginPt.z >> l.EndPt.x >> l.EndPt.y >> l.EndPt.z;
        (side?doc.m_RailVec2:doc.m_RailVec1).push_back(l);
    }
    for(int i=0;i<np;i++){ LiDPOINT p; cin>>p.x>>p.y>>p.z>>p.ClassCode;
        p.ClassCode = p.ClassCode==3?3:1; doc.m_LiDAR.push_back(p); }
    doc.OnHeadptsandtrackEvaluation();
    cout<<setprecision(17)<<"RESULT "<<stats.mh<<" "<<stats.sh<<" "<<stats.mv<<" "<<stats.sv<<" "
        <<stats.precision<<" "<<stats.recall<<" "<<stats.tp<<" "<<stats.fp<<" "<<stats.tn<<" "<<stats.fn<<"\n";
    vector<int> owners(np,-1); vector<Line3D*> lines;
    for(auto &l:doc.m_RailVec1) lines.push_back(&l);
    for(auto &l:doc.m_RailVec2) lines.push_back(&l);
    for(int i=0;i<lines.size();i++){
        for(auto p:lines[i]->TrkPts) owners[p-&doc.m_LiDAR[0]]=i;
        for(auto p:lines[i]->nonTrkPts) owners[p-&doc.m_LiDAR[0]]=i;
    }
    for(int i=0;i<np;i++){
        auto p=&doc.m_LiDAR[i]; int id=owners[i];
        cout<<"POINT "<<id<<" "<<(id<0?0:p->ClassCodeRe);
        if(id>=0) cout<<" "<<doc.DistanceBetweenPointAndLine2D(p,*lines[id])<<" "<<doc.DistanceOfHeightBetweenPointAndLine3D(p,*lines[id]);
        cout<<"\n";
    }
    for(auto l:lines){ l->GetLineParameter(); cout<<"POLY ";
        for(auto p:l->GetBufferVtxOfLine(.3)) cout<<p.x<<" "<<p.y<<" "; cout<<"\n"; }
}
'''
        self.directory = tempfile.TemporaryDirectory(prefix="railway-mls-evaluation-cpp-")
        source = Path(self.directory.name) / "oracle.cpp"
        self.executable = Path(self.directory.name) / "oracle"
        source.write_text(declarations + line + extract(header, "inline int pnpoly2(")
                          + klass + helpers + evaluation + main)
        result = subprocess.run(["g++", "-std=c++17", "-O0", "-ffp-contract=off", str(source),
                                 "-o", str(self.executable)], capture_output=True, text=True)
        if result.returncode:
            self.close()
            raise RuntimeError(result.stderr)

    def close(self):
        self.directory.cleanup()

    def evaluate(self, reference, left, right):
        values = [str(len(left)), str(len(right)), str(len(reference))]
        for rails in (left, right):
            values.extend(repr(float(v)) for v in rails.endpoints.flat)
        for xyz, label in zip(reference.xyz, reference.raw_classification):
            values.extend([*(repr(float(v)) for v in xyz), str(int(label))])
        result = subprocess.run([str(self.executable)], input=" ".join(values),
                                capture_output=True, text=True, check=True)
        output = {"points": [], "polygons": []}
        for line in result.stdout.splitlines():
            if line.startswith("RESULT "):
                output["metrics"] = [float(v) for v in line.split()[1:7]]
                output["counts"] = [int(v) for v in line.split()[7:]]
            elif line.startswith("POINT "):
                fields = line.split()[1:]
                output["points"].append([int(fields[0]), int(fields[1]),
                                         *([float(v) for v in fields[2:]] if len(fields)>2 else [np.nan,np.nan])])
            elif line.startswith("POLY "):
                output["polygons"].append(np.array([float(v) for v in line.split()[1:]]).reshape(4, 2))
        return output
