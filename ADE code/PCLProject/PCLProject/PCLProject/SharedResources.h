#pragma once

#include "Viewer.h"
#include <pcl/common/common_headers.h>

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

#include "LIDARPT.h"
#include "RailwayExtraction.h"


using namespace System;

ref class SharedResources
{
	SharedResources() {}
	SharedResources(const SharedResources%) { throw gcnew System::InvalidOperationException("singleton cannot be copy-constructed"); }
	static SharedResources m_instance;

public:
	static property SharedResources^ Instance { SharedResources^ get() { return %m_instance; } }

	cli::array<String ^> ^loadedFileNames;

};

