#include "MainForm.h"

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

#include <msclr/marshal.h>

#include <string>
#include <iostream>

#include "SharedData.h"
#include "VTKWindowController.h"
#include "InitialLocationForm.h"
#include "LIDARPT.h"

#include "DataConverstionForm.h"
#include "SharedResources.h"

#include "ExtractionParameterForm.h"
#include <boost/thread/thread.hpp>

#include "ExportOptionsForm.h"

using namespace System;
using namespace System::Windows::Forms;
using namespace LibLAS;
using namespace System::IO;
using namespace System::Threading;
using namespace std;
using namespace System::Runtime::InteropServices;

namespace boost {
	struct thread::dummy {};
}


System::Void PCLProject::MainForm::openToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	if (this->openLASFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Loading";

		cli::array<String ^> ^ fileNames = this->openLASFileDialog->FileNames;

		SharedResources::Instance->loadedFileNames = fileNames;
		this->listFiles(fileNames);

		String ^fileName = fileNames[0];
		this->loadFile(fileName);

		//Thread^ newThread1 = gcnew Thread(gcnew ParameterizedThreadStart(&LoadPointCloud));
		//newThread1->Start(this);
	}
}

System::Void PCLProject::MainForm::setInitalLocationToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{
	//this->railwayExtraction->OnRailextractionSetinit();
	//this->vtkWindowControl->InputInitialLocation();
	//InitLocationPick();

	POINT3D pt;

	InitialLocationForm ^dialogForm = gcnew InitialLocationForm();

	InitialLocation location = this->railwayExtraction->initialLocation;
	dialogForm->updateInputs(location);
	dialogForm->disableRetryButton();
	dialogForm->ShowDialog();

	//dialogForm->updateInputs(location.x, location.y, location.angle, location.mode);

	if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::Yes)
	{
		InitialLocation updatedLocation = dialogForm->getLocation();
		pt.x = updatedLocation.x;
		pt.y = updatedLocation.y;
		pt.z = updatedLocation.z;

		this->railwayExtraction->OnRailextractionSetinit(updatedLocation.mode, pt, updatedLocation.angle);
	}
	else if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::Retry)
	{

	}
	else if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::No)
	{

	}
}

System::Void PCLProject::MainForm::railwayTrackingToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{

	//boost::thread trackingThread(&RailwayExtraction::OnRailextractionTrackingRailway, this->railwayExtraction);
	//trackingThread.join();

	InitialLocation orginal_location = this->railwayExtraction->initialLocation;

	POINT3D pt;

	InitialLocation new_location = this->railwayExtraction->initialLocation;
	int mode = (new_location.mode + 2) % 4;
	pt.x = new_location.second_x;
	pt.y = new_location.second_y;
	pt.z = new_location.second_z;

	double angle = new_location.angle;
	angle = angle + 180 % 360;

	this->railwayExtraction->OnRailextractionSetinit(mode, pt, angle);

	this->railwayExtraction->OnRailextractionTrackingRailway();
	UpdateProgress(0, 0);

	// -- Reverse line
	this->railwayExtraction->ReserverVectors();

	mode = orginal_location.mode;
	pt.x = orginal_location.x;
	pt.y = orginal_location.y;
	pt.z = orginal_location.z;
	angle = orginal_location.angle;

	this->railwayExtraction->Setinitiallocation(mode, pt, angle);
	this->railwayExtraction->updateView(1);

	this->railwayExtraction->OnRailextractionTrackingRailway();
	UpdateProgress(0, 0);


	this->updateTrackedLayers();
}

System::Void PCLProject::MainForm::saveRailVectorToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{
	saveRailVectorAtIndex(0);
}

System::Void PCLProject::MainForm::saveRailVectorAtIndex(int index)
{
	if (this->saveCSVFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Saving";
		msclr::interop::marshal_context oMarshalContext;
		String^ fileName = this->saveCSVFileDialog->FileName;

		String^ fileNameWithTrackPoints = fileName->Substring(0, fileName->LastIndexOf(".")) + "_with_track_points.csv";
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
		const char* pFileNameWithTrackPoints = oMarshalContext.marshal_as<const char*>(fileNameWithTrackPoints);

		if (this->railwayExtraction->m_trackLayers.size() > 0) {
			this->txtReader->saveRailVector(pFileName, this->railwayExtraction->m_trackLayers[index]->m_RailVecCenter);
			this->txtReader->saveRailVector(pFileNameWithTrackPoints, this->railwayExtraction->m_trackLayers[index]->m_RailVecCenter,
				this->railwayExtraction->m_trackLayers[index]->m_RailVec1,
				this->railwayExtraction->m_trackLayers[index]->m_RailVec2);
		}

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}
}

System::Void PCLProject::MainForm::saveRailVectorWithIndexAtIndex(int index)
{
	if (this->saveCSVFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Saving";
		msclr::interop::marshal_context oMarshalContext;
		String^ fileName = this->saveCSVFileDialog->FileName;

		String^ fileNameWithTrackPoints = fileName->Substring(0, fileName->LastIndexOf(".")) + "_with_track_points.csv";
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
		const char* pFileNameWithTrackPoints = oMarshalContext.marshal_as<const char*>(fileNameWithTrackPoints);

		if (this->railwayExtraction->m_trackLayers.size() > 0) {
			this->txtReader->saveRailVectorWithIndex(pFileName, this->railwayExtraction->m_trackLayers[index]->m_RailVecCenter,
				this->railwayExtraction->m_trackLayers[index]->m_RailVec1,
				this->railwayExtraction->m_trackLayers[index]->m_RailVec2);
		}

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}
}

System::Void PCLProject::MainForm::savePointsToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{

}

System::Void PCLProject::MainForm::exitToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{

}

System::Void PCLProject::MainForm::showPointCloud(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud)
{
	this->vtkWindowControl->RemoveActors();
	this->vtkWindowControl->ReloadView();

	String ^ name = Thread::CurrentThread->Name;

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->SetDataTypeToDouble();

	vtkSmartPointer<vtkCellArray> cell = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkDoubleArray> depth = vtkSmartPointer<vtkDoubleArray>::New();
	depth->SetName("DepthArray");

	double min = point_cloud->points[0].z;
	double max = min;
	for (size_t i = 0; i < point_cloud->points.size(); i++)
	{
		double x = point_cloud->points[i].x;
		double y = point_cloud->points[i].y;
		double z = point_cloud->points[i].z;

		double point[3] = { x, y, z };
		vtkIdType id = points->InsertNextPoint(point);

		cell->InsertNextCell(1);
		cell->InsertCellPoint(id);
		depth->InsertNextValue(point_cloud->points[i].z);

		if (min > z) min = z;
		if (max < z) max = z;
	}

	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

	polyData->SetPoints(points);
	polyData->SetVerts(cell);
	polyData->GetPointData()->SetScalars(depth);
	polyData->GetPointData()->SetActiveScalars("DepthArray");

	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter = vtkSmartPointer<vtkCenterOfMass>::New();

	centerOfMassFilter->SetInputData(polyData);
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();

	double center[3];
	centerOfMassFilter->GetCenter(center);

	SharedData *data = SharedData::getInstance();
	data->centerOfMass[0] = center[0];
	data->centerOfMass[1] = center[1];
	data->centerOfMass[2] = center[2];

	vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
	translation->Translate(-1 * center[0], -1 * center[1], -1 * center[2]);

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(polyData);
	transformFilter->SetTransform(translation);
	transformFilter->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(polyData);
	mapper->SetInputConnection(transformFilter->GetOutputPort());
	mapper->SetColorModeToDefault();
	mapper->SetScalarRange(min, max);
	mapper->SetScalarVisibility(1);

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	//actor->GetProperty()->SetPointSize(5);

	this->vtkWindowControl->AddActor(actor);
}

System::Void PCLProject::MainForm::toolStripCameraTop_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraUp();
}

System::Void PCLProject::MainForm::toolStripCameraDown_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraDown();
}

System::Void PCLProject::MainForm::toolStripCameraLeft_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraLeft();
}

System::Void PCLProject::MainForm::toolStripCameraRight_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraRight();
}

System::Void PCLProject::MainForm::toolStripCameraFront_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraFront();
}

System::Void PCLProject::MainForm::toolStripCameraBack_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraBack();
}

System::Void PCLProject::MainForm::toolStripCameraIsometric_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->vtkWindowControl->MoveCameraIsometric();
}


System::Void PCLProject::MainForm::VTKMouseMove(int x, int y, int z)
{

}

System::Void PCLProject::MainForm::GenerateGrid()
{
	this->showPointCloud(this->railwayExtraction->point_cloud);
	this->railwayExtraction->generateGrid();
}

System::Void PCLProject::MainForm::LoadPointCloud(System::Object ^obj)
{
	MainForm ^thisForm = (MainForm^)obj;
	thisForm->GenerateGrid();
}

System::Void PCLProject::MainForm::InitLocationPick()
{
	ManagedPointsCallback^ callback = gcnew ManagedPointsCallback(this, &MainForm::ReceivedPoints);
	GCHandle gch = GCHandle::Alloc(callback);

	IntPtr stubPointer = Marshal::GetFunctionPointerForDelegate(callback);
	UnmanagedPointsCallback functionPointer = static_cast<UnmanagedPointsCallback>(stubPointer.ToPointer());

	GC::Collect();
	this->vtkWindowControl->controller->InitialLocation(functionPointer);

	GC::KeepAlive(callback);
	//gch.Free();
}

void PCLProject::MainForm::ReceivedPoints(double x1, double y1, double z1, double x2, double y2, double z2)
{
	cout << "xyz1: " << x1 << "," << y1 << "," << z1 << endl;
	cout << "xyz2: " << x2 << "," << y2 << "," << z2 << endl;

	POINT3D pt;
	pt.x = (x1 + x2) / 2;
	pt.y = (y1 + y2) / 2;
	pt.z = (z1 + z2) / 2;
	//pt.z = -9.2559631349317831e+61;
	
	//float angle = atan2(y2 - y1, x2 - x1);
	float angle = GetAngle(x1, y1, x2, y2);
	cout << "angle: " << angle << endl;

	int mode = 4;
	double dX = x2 - x1;
	double dY = y2 - y1;

	if (dX >= 0 && dY >= 0)
		mode = 1;
	else if (dX < 0 && dY >= 0)
		mode = 2;
	else if (dX < 0 && dY < 0)
		mode = 3;
	else
		mode = 4;
	
	//mode = mode % 4 + 2;

//	POINT3D pt;
//	int mode;
//	double angle;
//	pt.x = -728.289; pt.y = 1953.55; pt.z = 13.46; mode = 4; angle = 280;
//	pt.x = 513194.41; pt.y = 5458237.85; pt.z = 6.61; mode = 4; angle = 355;
	
	this->railwayExtraction->OnRailextractionSetinit(mode, pt, angle);

	InitialLocationForm ^dialogForm = gcnew InitialLocationForm();

	InitialLocation location = this->railwayExtraction->initialLocation;
	dialogForm->updateInputs(location);
	dialogForm->ShowDialog();

	
	//dialogForm->updateInputs(location.x, location.y, location.angle, location.mode);

	if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::Yes)
	{
		InitialLocation updatedLocation = dialogForm->getLocation();
		pt.x = updatedLocation.x;
		pt.y = updatedLocation.y;
		pt.z = updatedLocation.z;

		this->railwayExtraction->OnRailextractionSetinit(updatedLocation.mode, pt, updatedLocation.angle);
	}
	else if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::Retry)
	{
		InitLocationPick();
	} 
	else if (dialogForm->DialogResult == System::Windows::Forms::DialogResult::No)
	{

	}

}

double PCLProject::MainForm::GetAngle(double x1, double y1, double x2, double y2) {
	double dX = x2 - x1;
	double dY = y2 - y1;
	double dXY = sqrt(dX*dX + dY*dY);
	double fAngle;

	if (dY >= 0) {
		if (dX >= 0)
			fAngle = acos(fabs(dX / dXY));
		else
			fAngle = PI - acos(fabs(dX / dXY));
	}
	else {
		if (dX<0)
			fAngle = acos(fabs(dX / dXY));
		else
			fAngle = PI - acos(fabs(dX / dXY));
	}

	if (bIsNan(fAngle))
		fAngle = 0;

	return  fAngle * 180 / PI;
}

System::Void PCLProject::MainForm::openPTSFileToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	if (this->openPTSFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Loading";
		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->openPTSFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		this->txtReader = new TXTFileReader();
		pcl::PointCloud<PointXYZDouble>::Ptr point_cloud = txtReader->ReadData(pFileName);

		SharedData *sharedData = SharedData::getInstance();
		sharedData->point_cloud = point_cloud;

		//showPointCloud(point_cloud);

		this->railwayExtraction = new RailwayExtraction(point_cloud);
		this->railwayExtraction->windowController = this->vtkWindowControl->controller;

		this->railwayExtraction->readMaxMinFromTXTReader(txtReader);
		this->railwayExtraction->generateGrid();

		this->toolStripStatusLabel1->Text = "Status: Ready";

		this->railwayExtraction->updateView(1);

	}
}

System::Void PCLProject::MainForm::eCEFLASToUTMToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	if (this->openLASFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Loading";
		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->openLASFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		this->reader = new LASFileReader();
		pcl::PointCloud<PointXYZDouble>::Ptr point_cloud = reader->ReadData(pFileName);

		SharedData *sharedData = SharedData::getInstance();
		sharedData->point_cloud = point_cloud;

		showPointCloud(point_cloud);

		this->convertor = new DataConvertor(point_cloud);
		this->convertor->ConvertECEFToUTM(32);

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}


	if (this->savePTSFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Saving";
		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->savePTSFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		this->convertor->SaveConvertedPoints(pFileName);

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}
}

System::Void PCLProject::MainForm::uTMPTSToECEFPTSToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e)
{

	if (this->openPTSFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Loading";
		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->openPTSFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		this->txtReader = new TXTFileReader();
		pcl::PointCloud<PointXYZDouble>::Ptr point_cloud = txtReader->ReadData(pFileName);

		SharedData *sharedData = SharedData::getInstance();
		sharedData->point_cloud = point_cloud;

		showPointCloud(point_cloud);

		this->convertor = new DataConvertor(point_cloud);
		this->convertor->ConvertUTMToECEF(32, false);

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}


	if (this->savePTSFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		this->toolStripStatusLabel1->Text = "Status: Saving";
		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->savePTSFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		this->convertor->SaveConvertedPoints(pFileName);

		this->toolStripStatusLabel1->Text = "Status: Ready";
	}
}

System::Void PCLProject::MainForm::dataConversionToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	DataConverstionForm ^dialogForm = gcnew DataConverstionForm();

	dialogForm->ShowDialog();
}

System::Void PCLProject::MainForm::selectInitialLocationToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) 
{
	InitLocationPick();
}

System::Void PCLProject::MainForm::listFiles(cli::array<String^>^ fileNames)
{
	filesTreeView->Nodes->Clear();


	//TreeNode ^rootNode = gcnew TreeNode("Loaded Files");

	for (int i = 0; i < fileNames->Length; i++)
	{
		String ^fileName = Path::GetFileName(fileNames[i]);
		TreeNode ^node = gcnew TreeNode(fileName);
		filesTreeView->Nodes->Add(node);
	}

	//filesTreeView->Nodes->Add(rootNode);

}

System::Void PCLProject::MainForm::loadFile(String ^ fileName)
{
	this->Text = "ADE | " + fileName;

	msclr::interop::marshal_context oMarshalContext;
	const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
	
	pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;

	SharedData *sharedData = SharedData::getInstance();

	//const void * obj = sharedData->filesMap.get("yolo");

	RailwayExtraction *extractionInstance = (RailwayExtraction *) sharedData->filesMap.get(pFileName);

	if (extractionInstance != NULL)
	{
		this->railwayExtraction = extractionInstance;
		SharedData::getInstance()->point_cloud = this->railwayExtraction->point_cloud;
		this->railwayExtraction->updateView(3);

		this->updateTrackedLayers();
	}
	else
	{

		ProgressCallback^ callback = gcnew ProgressCallback(this, &MainForm::UpdateProgress);
		GCHandle gch = GCHandle::Alloc(callback);

		IntPtr stubPointer = Marshal::GetFunctionPointerForDelegate(callback);
		UnmanagedProgressCallback functionPointer = static_cast<UnmanagedProgressCallback>(stubPointer.ToPointer());

		String ^ ext = Path::GetExtension(fileName);

		GC::KeepAlive(callback);

		if (ext->Equals(".las") || ext->Equals(".laz"))
		{
			this->reader = new LASFileReader();
			point_cloud = reader->ReadData(pFileName, functionPointer);
		}
		else if (ext->Equals(".pts"))
		{
			this->txtReader = new TXTFileReader();
			point_cloud = txtReader->ReadData(pFileName, functionPointer);
		}
		else
		{
			return;
		}

		sharedData->point_cloud = point_cloud;

		//showPointCloud(point_cloud);

		RailwayExtraction *extraction = new RailwayExtraction(point_cloud);
		extraction->progressCallback = functionPointer;

		this->railwayExtraction = extraction;

		sharedData->filesMap.put(pFileName, extraction);

		this->railwayExtraction->windowController = this->vtkWindowControl->controller;

		if (ext->Equals(".las") || ext->Equals(".laz"))
		{
			this->railwayExtraction->updateMaxMinFromReader(reader, pFileName);
		}
		else if (ext->Equals(".pts"))
		{
			this->railwayExtraction->readMaxMinFromTXTReader(txtReader);
		}

		this->railwayExtraction->generateGrid();

		this->toolStripStatusLabel1->Text = "Status: Ready";
		this->toolStripProgressBar1->Value = 0;

		this->railwayExtraction->updateView(3);

		//gch.Free();
	}
}

System::Void PCLProject::MainForm::updateTrackedLayers()
{
	std::vector<TrackLayer *> layers = this->railwayExtraction->GetLayers();

	layersTreeView->Nodes->Clear();

	for (int i = 0; i < layers.size(); i++)
	{
		TrackLayer *layer = layers[i];
		TreeNode ^node = gcnew TreeNode("Rail " + (i + 1));
		node->Checked = true;

		node->ContextMenuStrip = layersContextMenuStrip;

		TreeNode ^childNode1 = gcnew TreeNode("Center Line");
		node->Nodes->Add(childNode1);
		childNode1->Checked = true;

		TreeNode ^childNode2 = gcnew TreeNode("Loop Cross");
		node->Nodes->Add(childNode2);
		childNode2->Checked = true;

		TreeNode ^childNode3 = gcnew TreeNode("Switch Box");
		node->Nodes->Add(childNode3);
		childNode3->Checked = true;

		TreeNode ^childNode4 = gcnew TreeNode("Toe Points");
		node->Nodes->Add(childNode4);
		childNode4->Checked = true;

		TreeNode ^childNode5 = gcnew TreeNode("Rail Track");
		node->Nodes->Add(childNode5);
		childNode5->Checked = true;

		layersTreeView->Nodes->Add(node);
	}
}

System::Void PCLProject::MainForm::filesTreeView_NodeMouseDoubleClick(System::Object ^ sender, System::Windows::Forms::TreeNodeMouseClickEventArgs ^ e) 
{
	TreeNode ^node = e->Node;
	String ^fileName = node->Text;

	cli::array<String ^> ^fileNames = SharedResources::Instance->loadedFileNames;

	int index = 0;

	for (int i = 0; i < fileNames->Length; i++)
	{
		if (Path::GetFileName(fileNames[i])->Equals(fileName))
		{
			index = i;
			break;
		}
	}

	String ^selectedFileName = fileNames[index];

	this->loadFile(selectedFileName);

}

System::Void PCLProject::MainForm::exportGWADToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {

	if (this->saveGWADFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		std::vector<TrackLayer *> layers = this->railwayExtraction->GetLayers();
		TreeNodeCollection ^collection = this->layersTreeView->Nodes;

		msclr::interop::marshal_context oMarshalContext;
		String ^fileName = this->saveGWADFileDialog->FileName;
		const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);

		std::ofstream ofs(pFileName);
		ofs.setf(ios::fixed);
		ofs.setf(ios::showpoint);

		ofs << "X\tY\tZ\tChainage\tElement\tParm1\tParm2" << endl;

		cout << "EXPORT --- \n";

		

		for (int i = 0; i < layers.size(); i++)
		{
			TrackLayer *layer = layers[i];
			TreeNode ^node = collection[i];
			const char* layerName = oMarshalContext.marshal_as<const char*>(node->Text);

			vector<GradeSect> grades = this->railwayExtraction->ComputeGrade(layer);

			cout << "layer: " << i << " Name: " << layerName << endl;

			double distance = 0;

			for (int j = 0; j < layer->m_RailVecCenter.size(); j++)
			{
				POINT3D pt1;
				POINT3D pt2 = layer->m_RailVecCenter[j];

				if (j != 0) 
				{
					pt1	= layer->m_RailVecCenter[j - 1];
					distance += this->railwayExtraction->DistanceBetweenPoints3D(pt1, pt2);
				}
				
				if (pt2.PtType == 1 || pt2.PtType == 2)
				{
					char * type = pt2.PtType == 1 ? "GR" : "SW";

					ofs << pt2.x << "\t" << pt2.y << "\t" << pt2.z << "\t" << layerName << "+" << distance << "\t" << type;

					if (pt2.PtType == 1)
					{
						for (int k = 0; k < grades.size(); k++)
						{
							GradeSect grade = grades[k];

							if (grade.BeginPt.IsEqual(pt2)) 
							{
								ofs << "\t" << "\t" << grade.grade;
							}
							else if (grade.EndPt.IsEqual(pt2))
							{
								ofs << "\t" << grade.grade << "\t";
							}
						}
					}

					ofs << endl;
				}
			}

			
		}

		ofs.close();

		printf("Save Done\n");
	}
}

System::Void PCLProject::MainForm::setParametersToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {

	ExtractionParameters parameters = this->railwayExtraction->getExtractionParameters();
	
	ExtractionParameterForm ^form = gcnew ExtractionParameterForm();
	form->updateValues(parameters);
	form->ShowDialog();

	if (form->DialogResult == System::Windows::Forms::DialogResult::Yes)
	{
		ExtractionParameters parameters = form->getValues();
		this->railwayExtraction->updateExtractionParameters(parameters);
	}
}

System::Void PCLProject::MainForm::computeGradeToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {
	
	std::vector<TrackLayer *> layers = this->railwayExtraction->GetLayers();

	for (int i = 0; i < layers.size(); i++)
	{
		TrackLayer *layer = layers[i];
		this->railwayExtraction->ComputeGrade(layer);
	}

}

System::Void PCLProject::MainForm::setPointCloudColorToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {
	ColorDialog^ colorDialog = gcnew ColorDialog;
	colorDialog->AllowFullOpen = true;

	// Update the text box color if the user clicks OK 
	if (colorDialog->ShowDialog() == ::System::Windows::Forms::DialogResult::OK)
	{
		//textBox1->ForeColor = MyDialog->Color;
		Color color = colorDialog->Color;
		//this->vtkWindowControl->UpdateColor(color);
		this->railwayExtraction->defaultColor_R = color.R;
		this->railwayExtraction->defaultColor_G = color.G;
		this->railwayExtraction->defaultColor_B = color.B;
		this->railwayExtraction->updateView(1);
	}
}

System::Void PCLProject::MainForm::exportOptionsToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {

	ExportOptionsForm ^dialogForm = gcnew ExportOptionsForm();

	dialogForm->ShowDialog();
}

System::Void PCLProject::MainForm::setBackgroundColorToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) {
	ColorDialog^ colorDialog = gcnew ColorDialog;
	colorDialog->AllowFullOpen = true;

	// Update the text box color if the user clicks OK 
	if (colorDialog->ShowDialog() == ::System::Windows::Forms::DialogResult::OK)
	{
		//textBox1->ForeColor = MyDialog->Color;
		Color color = colorDialog->Color;
		this->vtkWindowControl->UpdateColor(color);
	}
}

System::Void PCLProject::MainForm::toolIntensity_Click(System::Object ^ sender, System::EventArgs ^ e) 
{
	this->railwayExtraction->updateView(2);
}

System::Void PCLProject::MainForm::layersTreeView_EnabledChanged(System::Object ^ sender, System::EventArgs ^ e) {
	this->railwayExtraction->updateView(1, layersTreeView->Nodes);
}

System::Void PCLProject::MainForm::layersTreeView_AfterCheck(System::Object ^ sender, System::Windows::Forms::TreeViewEventArgs ^ e) {
	this->railwayExtraction->updateView(1, layersTreeView->Nodes);
}

System::Void PCLProject::MainForm::renameToolStripMenuItem_Click(System::Object ^ sender, System::EventArgs ^ e) 
{
	ToolStripMenuItem ^item = safe_cast<ToolStripMenuItem ^>(sender);

	msclr::interop::marshal_context oMarshalContext;
	String ^fileName = this->selectedNode->Text;
	const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
	std::cout << "rename clicked: " << pFileName;

	this->selectedNode->BeginEdit();
}

System::Void PCLProject::MainForm::SaveVectorToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
	ToolStripMenuItem^ item = safe_cast<ToolStripMenuItem^>(sender);

	int index = this->selectedNode->Index;

	msclr::interop::marshal_context oMarshalContext;
	String^ fileName = this->selectedNode->Text;
	const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
	std::cout << "save clicked: " << pFileName << " at index: " <<index;
	
	saveRailVectorAtIndex(index);
}

System::Void PCLProject::MainForm::saveRailVectorWithIndexToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e)
{
	saveRailVectorWithIndexAtIndex(0);
}


System::Void PCLProject::MainForm::layersTreeView_NodeMouseClick(System::Object ^ sender, System::Windows::Forms::TreeNodeMouseClickEventArgs ^ e) 
{
	if (e->Button == System::Windows::Forms::MouseButtons::Right)
	{
		this->selectedNode = e->Node;
	}
}

void PCLProject::MainForm::UpdateProgress(double readPoints, double totalPoints)
{
	this->toolStripProgressBar1->Minimum = 0;
	this->toolStripProgressBar1->Maximum = totalPoints;
	this->toolStripProgressBar1->Value = readPoints;
}
