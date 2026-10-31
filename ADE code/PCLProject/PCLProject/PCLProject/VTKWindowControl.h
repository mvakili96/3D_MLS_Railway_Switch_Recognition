#pragma once
#include "VTKWindowController.h"
#include <vtkActor.h>
#include <vtkSmartPointer.h>

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;


namespace PCLProject {

	/// <summary>
	/// Summary for VTKWindowControl
	/// </summary>
	public ref class VTKWindowControl : public System::Windows::Forms::UserControl
	{
	public:
		VTKWindowControl(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~VTKWindowControl()
		{
			if (components)
			{
				delete components;
			}

			delete controller;
		}

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
		}
#pragma endregion

		public: VTKWindowController* controller;
		public: System::Windows::Forms::ToolStripStatusLabel ^mousePositionLabel;

		protected: virtual void OnCreateControl() override;
		protected: virtual void OnResize(System::EventArgs^ e) override;

		protected: virtual void OnMouseDown(System::Windows::Forms::MouseEventArgs^ e) override;
		protected: virtual void OnMouseUp(System::Windows::Forms::MouseEventArgs^ e) override;
		protected: virtual void OnMouseMove(System::Windows::Forms::MouseEventArgs^ e) override;
		protected: virtual void OnMouseWheel(System::Windows::Forms::MouseEventArgs^ e) override;

		public: void AddActor(vtkSmartPointer<vtkActor> actor);
		public: void RemoveActors();
		public: void ReloadView();
		public: void addCoordinateSystem(double scale, float x, float y, float z, const std::string& id, int viewport);

		//Camera Position
		public: void MoveCameraUp();
		public:	void MoveCameraDown();
		public: void MoveCameraLeft();
		public: void MoveCameraRight();
		public: void MoveCameraFront();
		public: void MoveCameraBack();
		public: void MoveCameraIsometric();

		//Coloring
		public: void UpdateColor(Color color);

	};
}
