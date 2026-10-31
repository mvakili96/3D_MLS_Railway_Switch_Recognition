#pragma once

#include "MainForm.h"

#define generic __identifier(generic)

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#undef generic

#include <vtkTextActor.h>

using namespace System;
using namespace System::Windows::Forms;

[STAThread]
int main()
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);

	PCLProject::MainForm form;
	Application::Run(%form);

	return 0;
}

void pcl::visualization::PCLVisualizer::FPSCallback::Execute(vtkObject* caller, unsigned long, void*)
{
	vtkRenderer *ren = reinterpret_cast<vtkRenderer *> (caller);
	float fps = 1.0f / static_cast<float> (ren->GetLastRenderTimeInSeconds());
	char buf[128];
	sprintf(buf, "%.1f FPS", fps);
	actor->SetInput(buf);
}
