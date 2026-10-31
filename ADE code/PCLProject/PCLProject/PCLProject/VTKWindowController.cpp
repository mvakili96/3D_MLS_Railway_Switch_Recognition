#include "VTKWindowController.h"
#include <vtkAxes.h>
#include <vtkFloatArray.h>
#include <vtkTubeFilter.h>
#include <vtkLODActor.h>
#include <vtkRendererCollection.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCoordinate.h>
#include <vcclr.h>
#include <boost\thread.hpp>
#include "SharedData.h"
#include <pcl/common/common_headers.h>
#include "PointXYZDouble.h"
#include <iostream>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPointPicker.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

// Define interaction style
class VMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
private:
	unsigned int NumberOfClicks;
	int PreviousPosition[2];
	int ResetPixelDistance;
	bool getNewPoints;
	double point1[3];
	double point2[3];

public:
	static VMouseInteractorStyle* New();
	gcroot<System::Windows::Forms::ToolStripStatusLabel ^> mouseLabel;
	UnmanagedPointsCallback callback;

	VMouseInteractorStyle() : NumberOfClicks(0), ResetPixelDistance(5)
	{
		this->PreviousPosition[0] = 0;
		this->PreviousPosition[1] = 0;
		this->getNewPoints = false;

		point1[0] = -1;
		point1[1] = -1;
		point1[2] = -1;

		point2[0] = -1;
		point2[1] = -1;
		point2[2] = -1;
	}

	void SetNewPoints()
	{
		getNewPoints = true;
	}

	virtual void OnMouseMove()
	{
		//std::cout << "Mouse Move." << std::endl;
		int x = this->Interactor->GetEventPosition()[0];
		int y = this->Interactor->GetEventPosition()[1];
		int z = this->Interactor->GetEventPosition()[2];

		//mouseLabel->Text =  "XYZ: " + x + "," + y + "," + z;
		vtkSmartPointer<vtkCoordinate> coordinate =
			vtkSmartPointer<vtkCoordinate>::New();
		coordinate->SetCoordinateSystemToDisplay();
		coordinate->SetValue(x, y, 0);

		// This doesn't produce the right value if the sphere is zoomed in???
		double* world = coordinate->GetComputedWorldValue(this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
		//std::cout << "World coordinate: " << world[0] << ", " << world[1] << ", " << world[2] << std::endl;
		
		pcl::PointCloud<PointXYZDouble>::Ptr point_cloud = SharedData::getInstance()->point_cloud;

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		points->SetDataTypeToDouble();

		vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
		polyData->SetPoints(points);

		points->InsertNextPoint(world[0], world[1], world[2]);

		double center[3];
		center[0] = SharedData::getInstance()->centerOfMass[0];
		center[1] = SharedData::getInstance()->centerOfMass[1];
		center[2] = SharedData::getInstance()->centerOfMass[2];

		vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
		translation->Translate(-1 * center[0], -1 * center[1], -1 * center[2]);
		translation->Inverse();

		vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformFilter->SetInputData(polyData);
		transformFilter->SetTransform(translation);
		transformFilter->Update();

		vtkSmartPointer<vtkPolyData> outPolyData = transformFilter->GetOutput();
		double *point = outPolyData->GetPoint(0);

		mouseLabel->Text = "XYZ: " + point[0] + ", " + point[1] + ", " + point[2];

		// Forward events
		vtkInteractorStyleTrackballCamera::OnMouseMove();

	}

	virtual void OnLeftButtonDown()
	{
		
		//std::cout << "Pressed left mouse button." << std::endl;
		this->NumberOfClicks++;
		//std::cout << "NumberOfClicks = " << this->NumberOfClicks << std::endl;
		int pickPosition[2];
		this->GetInteractor()->GetEventPosition(pickPosition);

		int xdist = pickPosition[0] - this->PreviousPosition[0];
		int ydist = pickPosition[1] - this->PreviousPosition[1];

		this->PreviousPosition[0] = pickPosition[0];
		this->PreviousPosition[1] = pickPosition[1];

		int moveDistance = (int)sqrt((double)(xdist*xdist + ydist*ydist));

		// Reset numClicks - If mouse moved further than resetPixelDistance
		if (moveDistance > this->ResetPixelDistance)
		{
			this->NumberOfClicks = 1;
		}

		if (this->NumberOfClicks == 2)
		{

			/*std::cout << std::endl << std::endl;
			std::cout << "----- picked -----" << std::endl;

			std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0] << " " << this->Interactor->GetEventPosition()[1] << std::endl;
			this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
				this->Interactor->GetEventPosition()[1],
				0,  // always zero.
				this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			double picked[3];
			this->Interactor->GetPicker()->GetPickPosition(picked);
			std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;
			// Forward events
			*/

			int x = this->Interactor->GetEventPosition()[0];
			int y = this->Interactor->GetEventPosition()[1];
			int z = this->Interactor->GetEventPosition()[2];

			//mouseLabel->Text =  "XYZ: " + x + "," + y + "," + z;
			vtkSmartPointer<vtkCoordinate> coordinate =
				vtkSmartPointer<vtkCoordinate>::New();
			coordinate->SetCoordinateSystemToDisplay();
			coordinate->SetValue(x, y, 0);

			std::cout << "----- original -----" << std::endl;
			double* world = coordinate->GetComputedWorldValue(this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
			//double* world = picked;

			pcl::PointCloud<PointXYZDouble>::Ptr point_cloud = SharedData::getInstance()->point_cloud;

			vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
			points->SetDataTypeToDouble();

			vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
			polyData->SetPoints(points);

			points->InsertNextPoint(world[0], world[1], world[2]);

			double center[3];
			center[0] = SharedData::getInstance()->centerOfMass[0];
			center[1] = SharedData::getInstance()->centerOfMass[1];
			center[2] = SharedData::getInstance()->centerOfMass[2];

			vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
			translation->Translate(-1 * center[0], -1 * center[1], -1 * center[2]);
			translation->Inverse();

			vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformFilter->SetInputData(polyData);
			transformFilter->SetTransform(translation);
			transformFilter->Update();

			vtkSmartPointer<vtkPolyData> outPolyData = transformFilter->GetOutput();
			double *point = outPolyData->GetPoint(0);

			// This doesn't produce the right value if the sphere is zoomed in???

			std::cout << "Double Clicked:   " << world[0] * center[0] << ", " << world[1] * center[1] << ", " << world[2] * center[2] << std::endl;
			std::cout << "Original Clicked: " << world[0] << ", " << world[1] << ", " << world[2] << std::endl;
			std::cout << "T Clicked:        " << point[0] << ", " << point[1] << ", " << point[2] << std::endl;
			

			

			this->NumberOfClicks = 0;

			if (getNewPoints)
			{
				if (point1[0] == -1 &&
					point1[1] == -1 &&
					point2[2] == -1)
				{
					point1[0] = point[0];
					point1[1] = point[1];
					point1[2] = point[2];
				}
				else
				{
					callback(point1[0], point1[1], point1[2], point[0], point[1], point[2]);
					getNewPoints = false;
					point1[0] = -1;
					point1[1] = -1;
					point1[2] = -1;
				}
			}
		}

		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}
};

vtkStandardNewMacro(VMouseInteractorStyle);

VTKWindowController::VTKWindowController(System::Windows::Forms::UserControl^ control, System::Windows::Forms::ToolStripStatusLabel^ mousePositionLabel)
{
	renderer = vtkSmartPointer<vtkRenderer>::New();

	renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);

	// Add the actor to the scene
	//renderer->AddActor(actor);
	renderer->SetBackground(52 / 255.0, 152 / 255.0, 219 / 255.0); 
	renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	vtkSmartPointer<VMouseInteractorStyle> style = vtkSmartPointer<VMouseInteractorStyle>::New();
	renderWindowInteractor->SetInteractorStyle(style);
	style->mouseLabel = mousePositionLabel;
	//style->SetCurrentStyleToTrackballCamera();


	mapRenderer = vtkSmartPointer<vtkRenderer>::New();
	mapRenderer->SetBackground(52 / 255.0, 152 / 255.0, 219 / 255.0);
	
	renderWindow->AddRenderer(mapRenderer);
	mapRenderer->SetViewport(0.7, 0.7,1,1);

	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderWindow->SetSize(control->Width, control->Height);

	renderWindow->SetParentId(control->Handle.ToPointer()); 

	renderWindow->Render();
}


VTKWindowController::~VTKWindowController()
{
	//renderWindow->Delete();
	//renderWindowInteractor->Delete();
	//renderer->Delete();
}

void VTKWindowController::Resize(System::Windows::Forms::UserControl ^ control, System::EventArgs ^ e)
{
	renderWindow->SetSize(control->Width, control->Height);
}

void VTKWindowController::MouseDown(System::Windows::Forms::UserControl ^ control, System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->renderWindowInteractor)
	{
		this->renderWindowInteractor->SetEventInformationFlipY(e->X, e->Y, 0, 0, 0, e->Clicks, NULL);

		switch (e->Button)
		{
		case System::Windows::Forms::MouseButtons::Left:
			this->renderWindowInteractor->LeftButtonPressEvent();
			break;

		case System::Windows::Forms::MouseButtons::Middle:
			this->renderWindowInteractor->MiddleButtonPressEvent();
			break;

		case System::Windows::Forms::MouseButtons::Right:
			this->renderWindowInteractor->RightButtonPressEvent();
			break;
		}
	}
}

void VTKWindowController::MouseUp(System::Windows::Forms::UserControl ^ control, System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->renderWindowInteractor)
	{
		this->renderWindowInteractor->SetEventInformationFlipY( e->X, e->Y, 0, 0, 0, e->Clicks, NULL);

		switch (e->Button)
		{
		case System::Windows::Forms::MouseButtons::Left:
			this->renderWindowInteractor->LeftButtonReleaseEvent();
			break;

		case System::Windows::Forms::MouseButtons::Middle:
			this->renderWindowInteractor->MiddleButtonReleaseEvent();
			break;

		case System::Windows::Forms::MouseButtons::Right:
			this->renderWindowInteractor->RightButtonReleaseEvent();
			break;
		}
	}
}

void VTKWindowController::MouseMove(System::Windows::Forms::UserControl ^ control, System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->renderWindowInteractor)
	{
		this->renderWindowInteractor->SetEventInformationFlipY(e->X, e->Y, 0, 0, 0, e->Clicks, NULL);
		this->renderWindowInteractor->MouseMoveEvent();
	}
}

void VTKWindowController::MouseWheel(System::Windows::Forms::UserControl ^ control, System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->renderWindowInteractor)
	{
		this->renderWindowInteractor->SetEventInformationFlipY(e->X, e->Y, 0, 0, 0, e->Clicks, NULL);
		if (e->Delta > 0)
		{
			this->renderWindowInteractor->MouseWheelForwardEvent();
		}
		else
		{
			this->renderWindowInteractor->MouseWheelBackwardEvent();
		}
	}
}

void VTKWindowController::AddActor(vtkSmartPointer<vtkActor> actor)
{
	renderer->AddActor(actor);
	renderer->ResetCamera();
	renderer->Render();

	mapRenderer->AddActor(actor);
	mapRenderer->ResetCamera();
	mapRenderer->Render();

	renderWindow->Render();
}

void VTKWindowController::AddActorWithoutReloading(vtkSmartPointer<vtkActor> actor)
{
	renderer->AddActor(actor);
}

void VTKWindowController::AddActors(vtkSmartPointer<vtkActorCollection> actors)
{
	vtkSmartPointer<vtkActor> actor;
	while (actor = actors->GetNextActor())
	{
		renderer->AddActor(actor);
		mapRenderer->AddActor(actor);
	}

	renderer->ResetCamera();
	renderer->Render();

	mapRenderer->ResetCamera();
	mapRenderer->Render();

	renderWindow->Render();
}

void VTKWindowController::ReloadView()
{
	renderer->Render();
	mapRenderer->Render();
	renderWindow->Render();
}

void VTKWindowController::RemoveActors()
{
	/*vtkSmartPointer<vtkActorCollection> actors = renderer->GetActors();

	vtkSmartPointer<vtkActor> oldActor;

	while (oldActor = actors->GetNextActor())
	{
		renderer->RemoveActor(oldActor);
	}*/

	renderer->RemoveAllViewProps();
	mapRenderer->RemoveAllViewProps();
}

void VTKWindowController::addCoordinateSystem(double scale, float x, float y, float z, const std::string& id, int viewport)
{
	if (scale <= 0.0)
		scale = 1.0;
	vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New();
	axes->SetOrigin(0, 0, 0);
	axes->SetScaleFactor(scale);
	axes->Update();

	vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New();
	axes_colors->Allocate(6);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(1.0);
	axes_colors->InsertNextValue(1.0);

	vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput();
	axes_data->GetPointData()->SetScalars(axes_colors);

	vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New();
#if VTK_MAJOR_VERSION < 6
	axes_tubes->SetInput(axes_data);
#else
	axes_tubes->SetInputData(axes_data);
#endif
	axes_tubes->SetRadius(axes->GetScaleFactor() / 50.0);
	axes_tubes->SetNumberOfSides(6);

	vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	axes_mapper->SetScalarModeToUsePointData();
#if VTK_MAJOR_VERSION < 6
	axes_mapper->SetInput(axes_tubes->GetOutput());
#else
	axes_mapper->SetInputConnection(axes_tubes->GetOutputPort());
#endif

	vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New();
	axes_actor->SetMapper(axes_mapper);
	axes_actor->SetPosition(x, y, z);

	this->renderer->AddActor(axes_actor);
}

void VTKWindowController::resetViewDirection(double pos_x, double pos_y, double pos_z, double look_x, double look_y, double look_z, double up_x, double up_y, double up_z)
{
	if (NULL != this->renderWindowInteractor)
	{
		// vtk Element /////////////////////////////////////////////////////////
		vtkRenderWindowInteractor *rwi = this->renderWindowInteractor;
		vtkRenderer *renderer = this->renderWindow->GetRenderers()->GetFirstRenderer();
		vtkCamera *camera = renderer->GetActiveCamera();

		double pos[3] = { pos_x, pos_y, pos_z };
		double focal_point[3] = { look_x, look_y, look_z };
		double view_up[3] = { up_x, up_y, up_z };

		camera->SetPosition(pos);
		camera->SetFocalPoint(focal_point);
		camera->SetViewUp(view_up);

		// Orhthogonalize View Up //////////////////////////////////////////////////
		//camera->OrthogonalizeViewUp();

		renderer->ResetCameraClippingRange();
		rwi->Render();

	}
}

void VTKWindowController::MoveCameraUp()
{
	this->resetViewDirection(0, 0, 100, 0, 0, 1, 0, 1, 0);
}

void VTKWindowController::MoveCameraDown()
{
	this->resetViewDirection(0, 0, -100, 0, 0, -1, 0, 1, 0);
}

void VTKWindowController::MoveCameraLeft()
{
	this->resetViewDirection(-100, 0, 0, -1, 0, 0, 0, 0, 1);
}

void VTKWindowController::MoveCameraRight()
{
	this->resetViewDirection(100, 0, 0, 1, 0, 0, 0, 0, 1);
}

void VTKWindowController::MoveCameraFront()
{
	this->resetViewDirection(0, 100, 0, 0, 1, 0, 0, 0, 1);
}

void VTKWindowController::MoveCameraBack()
{
	this->resetViewDirection(0, -100, 0, 0, -1, 0, 0, 0, 1);
}

void VTKWindowController::MoveCameraIsometric()
{
	if (NULL != this->renderWindowInteractor)
	{

		// vtk Element /////////////////////////////////////////////////////////
		vtkRenderWindowInteractor *rwi = this->renderWindowInteractor;
		vtkRenderer *renderer = this->renderWindow->GetRenderers()->GetFirstRenderer();
		vtkCamera *camera = renderer->GetActiveCamera();

		camera->SetPosition(100, 100, 100);
		camera->SetFocalPoint(0, 0, 0);

		// Orhthogonalize View Up //////////////////////////////////////////////////
		camera->OrthogonalizeViewUp();
		renderer->ResetCameraClippingRange();

		rwi->Render();
	}
}

void VTKWindowController::InitialLocation(UnmanagedPointsCallback callback)
{
	//callback(0, 1, 2, 3, 4, 5);
	
	if (NULL != this->renderWindowInteractor)
	{
		VMouseInteractorStyle *style = (VMouseInteractorStyle *)this->renderWindowInteractor->GetInteractorStyle();
		style->SetNewPoints();
		style->callback = callback;
	}
}

void VTKWindowController::UpdateColor(int r, int g, int b)
{
	if (NULL != this->renderer) 
	{
		renderer->SetBackground(r / 255.0, g / 255.0, b / 255.0);
		ReloadView();
	}
}
