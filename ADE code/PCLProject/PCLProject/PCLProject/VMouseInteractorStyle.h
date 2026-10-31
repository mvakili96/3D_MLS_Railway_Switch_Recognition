#ifndef _VMOUSEINTERACTORSTYLE_H
#define _VMOUSEINTERACTORSTYLE_H


#include "typedef1.h"

// Define interaction style
class VMouseInteractorStyle : public typedef1 {
  private:
    unsigned int NumberOfClicks;

    int PreviousPosition[2];

    int ResetPixelDistance;

    bool getNewPoints;

    double point1[3];

    double point2[3];


  public:
    static VMouseInteractorStyle * New();

    gcroot<System::Windows::Forms::ToolStripStatusLabel ^> mouseLabel;

    UnmanagedPointsCallback callback;

    inline VMouseInteractorStyle() : NumberOfClicks(0), ResetPixelDistance(5)
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
    	};

    inline void SetNewPoints() {
    		getNewPoints = true;
    	};

    inline virtual void OnMouseMove() {
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
    	};

    inline virtual void OnLeftButtonDown() {
    		
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
    			int x = this->Interactor->GetEventPosition()[0];
    			int y = this->Interactor->GetEventPosition()[1];
    			int z = this->Interactor->GetEventPosition()[2];
    
    			//mouseLabel->Text =  "XYZ: " + x + "," + y + "," + z;
    			vtkSmartPointer<vtkCoordinate> coordinate =
    				vtkSmartPointer<vtkCoordinate>::New();
    			coordinate->SetCoordinateSystemToDisplay();
    			coordinate->SetValue(x, y, 0);
    
    			double* world = coordinate->GetComputedWorldValue(this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
    
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
    			
    			//std::cout << "Double Clicked:   " << world[0] * center[0] << ", " << world[1] * center[1] << ", " << world[2] * center[2] <<std::endl;
    			//std::cout << "Original Clicked: " << world[0] << ", " << world[1] << ", " << world[2] << std::endl;
    			//std::cout << "T Clicked:        " << point[0] << ", " << point[1] << ", " << point[2] << std::endl;
    
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
    	};

};
#endif
