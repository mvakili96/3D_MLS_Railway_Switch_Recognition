#pragma once

#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkAutoInit.h> 
#include <vtkInteractorStyleSwitch.h>
#include <vtkPointSource.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2)

typedef void(*UnmanagedPointsCallback)(double x1, double y1, double z1, double x2, double y2, double z3);

class VTKWindowController
{
public:
	VTKWindowController(System::Windows::Forms::UserControl^ control, System::Windows::Forms::ToolStripStatusLabel^ mousePositionLabels);
	~VTKWindowController();

private:
	vtkSmartPointer<vtkRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderer> mapRenderer;

	void resetViewDirection(double pos_x, double pos_y, double pos_z, double look_x, double look_y, double look_z, double up_x, double up_y, double up_z);

public:
	void Resize(System::Windows::Forms::UserControl^ control, System::EventArgs ^ e);

	void MouseDown(System::Windows::Forms::UserControl^ control, System::Windows::Forms::MouseEventArgs ^ e);
	void MouseUp(System::Windows::Forms::UserControl^ control, System::Windows::Forms::MouseEventArgs ^ e);
	void MouseMove(System::Windows::Forms::UserControl^ control, System::Windows::Forms::MouseEventArgs ^ e);
	void MouseWheel(System::Windows::Forms::UserControl^ control, System::Windows::Forms::MouseEventArgs ^ e);

	//Add Actor
	void AddActor(vtkSmartPointer<vtkActor> actor);
	void AddActorWithoutReloading(vtkSmartPointer<vtkActor> actor);
	void ReloadView();
	void AddActors(vtkSmartPointer<vtkActorCollection> actors);
	void RemoveActors();
	void addCoordinateSystem(double scale, float x, float y, float z, const std::string& id, int viewport);


	//Camera Position
	void MoveCameraUp();
	void MoveCameraDown();
	void MoveCameraLeft();
	void MoveCameraRight();
	void MoveCameraFront();
	void MoveCameraBack();
	void MoveCameraIsometric();

	void InitialLocation(UnmanagedPointsCallback callback);

	void UpdateColor(int r, int g, int b);
};

