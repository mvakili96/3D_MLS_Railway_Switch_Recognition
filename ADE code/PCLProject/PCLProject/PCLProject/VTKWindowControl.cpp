#include "VTKWindowControl.h"


void PCLProject::VTKWindowControl::OnCreateControl()
{
	this->controller = new VTKWindowController(this, mousePositionLabel);
}

void PCLProject::VTKWindowControl::OnResize(System::EventArgs ^ e)
{
	if (NULL != this->controller)
	{
		this->controller->Resize(this, e);
	}
}

void PCLProject::VTKWindowControl::OnMouseDown(System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->controller)
	{
		this->controller->MouseDown(this, e);
	}
}

void PCLProject::VTKWindowControl::OnMouseUp(System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->controller)
	{
		this->controller->MouseWheel(this, e);
	}
}

void PCLProject::VTKWindowControl::OnMouseMove(System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->controller)
	{
		this->controller->MouseMove(this, e);
	}
}

void PCLProject::VTKWindowControl::OnMouseWheel(System::Windows::Forms::MouseEventArgs ^ e)
{
	if (NULL != this->controller)
	{
		this->controller->MouseWheel(this, e);
	}
}

void PCLProject::VTKWindowControl::AddActor(vtkSmartPointer<vtkActor> actor)
{
	if (NULL != this->controller)
	{
		this->controller->AddActor(actor);
	}
}

void PCLProject::VTKWindowControl::RemoveActors()
{
	if (NULL != this->controller)
	{
		this->controller->RemoveActors();
	}
}

void PCLProject::VTKWindowControl::ReloadView()
{
	if (NULL != this->controller)
	{
		this->controller->ReloadView();
	}
}

void PCLProject::VTKWindowControl::addCoordinateSystem(double scale, float x, float y, float z, const std::string& id, int viewport)
{
	this->controller->addCoordinateSystem(scale, x, y, z, id, viewport);
}

void PCLProject::VTKWindowControl::MoveCameraUp()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraUp();
	}
}

void PCLProject::VTKWindowControl::MoveCameraDown()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraDown();
	}
}


void PCLProject::VTKWindowControl::MoveCameraLeft()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraLeft();
	}
}


void PCLProject::VTKWindowControl::MoveCameraRight()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraRight();
	}
}


void PCLProject::VTKWindowControl::MoveCameraFront()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraFront();
	}
}


void PCLProject::VTKWindowControl::MoveCameraBack()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraBack();
	}
}

void PCLProject::VTKWindowControl::MoveCameraIsometric()
{
	if (NULL != this->controller)
	{
		this->controller->MoveCameraIsometric();
	}
}

void PCLProject::VTKWindowControl::UpdateColor(Color color)
{
	if (NULL != this->controller)
	{
		this->controller->UpdateColor(color.R, color.G, color.B);
	}
}


