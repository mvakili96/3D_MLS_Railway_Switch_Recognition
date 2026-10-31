#pragma once
#include "VTKWindowControl.h"
#include "Viewer.h"
#include <pcl/common/common_headers.h>
#include "PointXYZDouble.h"
#include "LASFileReader.h"
#include "LIDARPT.h"
#include "RailwayExtraction.h"
#include "DataConvertor.h"
#include "TXTFileReader.h"

namespace PCLProject {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for MainForm
	/// </summary>
	public ref class MainForm : public System::Windows::Forms::Form
	{
	public:
		MainForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			AddCustomControls();
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}

			if (reader)
			{
				delete reader;
			}
		}
	private: System::Windows::Forms::Panel^  panel1;
	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	private: System::Windows::Forms::ToolStripMenuItem^  fileToolStripMenuItem;

	private: System::Windows::Forms::ToolStripMenuItem^  openToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator;


	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator1;



	private: System::Windows::Forms::ToolStripMenuItem^  exitToolStripMenuItem;


















	private: System::Windows::Forms::StatusStrip^  statusStrip1;
	private: System::Windows::Forms::OpenFileDialog^  openLASFileDialog;
	private: System::Windows::Forms::OpenFileDialog^  openPTSFileDialog;
	private: System::Windows::Forms::SaveFileDialog^  savePTSFileDialog;
	private: System::Windows::Forms::SaveFileDialog^  saveCSVFileDialog;
	private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel1;
	private: System::Windows::Forms::ToolStripMenuItem^  setInitalLocationToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  trackingRailwayToolStripMenuItem;


	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator2;
	private: System::Windows::Forms::ToolStripMenuItem^  saveRailVectorToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  savePointsToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  editToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  viewToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  optionsToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  helpToolStripMenuItem;
	private: System::Windows::Forms::ToolStrip^  toolStrip1;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton1;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton2;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton3;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton4;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton5;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton6;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton7;
	private: System::Windows::Forms::TabPage^  tabPage2;

	private: System::Windows::Forms::TabPage^  tabPage1;
	private: System::Windows::Forms::TabControl^  tabControl1;
	private: System::Windows::Forms::ToolStripProgressBar^  toolStripProgressBar1;
	private: System::Windows::Forms::TrackBar^  trackBar1;
	private: System::Windows::Forms::TreeView^  filesTreeView;

	private: System::Windows::Forms::TreeView^  layersTreeView;
	private: System::Windows::Forms::TreeView^  treeView2;

	private: System::Windows::Forms::ToolStripMenuItem^  dataConversionToolStripMenuItem;



	private: System::Windows::Forms::ToolStripMenuItem^  setBackgroundColorToolStripMenuItem;
	private: System::Windows::Forms::ToolStripButton^  toolStripButton8;
	private: System::Windows::Forms::ContextMenuStrip^  layersContextMenuStrip;


	private: System::Windows::Forms::ToolStripMenuItem^  renameToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^  selectInitialLocationToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^  exportGWADToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^  setParametersToolStripMenuItem;

private: System::Windows::Forms::SaveFileDialog^  saveGWADFileDialog;
private: System::Windows::Forms::SaveFileDialog^  saveFileDialog1;
private: System::Windows::Forms::ToolStripMenuItem^  setPointCloudColorToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^  exportOptionsToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^ saveVectorToolStripMenuItem;
private: System::Windows::Forms::ToolStripMenuItem^ saveRailVectorWithIndexToolStripMenuItem;

	private: System::ComponentModel::IContainer^  components;




















	protected:


	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MainForm::typeid));
			System::Windows::Forms::TreeNode^ treeNode1 = (gcnew System::Windows::Forms::TreeNode(L"file1.las"));
			System::Windows::Forms::TreeNode^ treeNode2 = (gcnew System::Windows::Forms::TreeNode(L"file2.pts"));
			System::Windows::Forms::TreeNode^ treeNode3 = (gcnew System::Windows::Forms::TreeNode(L"file3.laz"));
			System::Windows::Forms::TreeNode^ treeNode4 = (gcnew System::Windows::Forms::TreeNode(L"Project 1", gcnew cli::array< System::Windows::Forms::TreeNode^  >(3) {
				treeNode1,
					treeNode2, treeNode3
			}));
			System::Windows::Forms::TreeNode^ treeNode5 = (gcnew System::Windows::Forms::TreeNode(L"Node4"));
			System::Windows::Forms::TreeNode^ treeNode6 = (gcnew System::Windows::Forms::TreeNode(L"Node2", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) { treeNode5 }));
			System::Windows::Forms::TreeNode^ treeNode7 = (gcnew System::Windows::Forms::TreeNode(L"Node0", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) { treeNode6 }));
			System::Windows::Forms::TreeNode^ treeNode8 = (gcnew System::Windows::Forms::TreeNode(L"Node5"));
			System::Windows::Forms::TreeNode^ treeNode9 = (gcnew System::Windows::Forms::TreeNode(L"Node3", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) { treeNode8 }));
			System::Windows::Forms::TreeNode^ treeNode10 = (gcnew System::Windows::Forms::TreeNode(L"Node1", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) { treeNode9 }));
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->fileToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->selectInitialLocationToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->setInitalLocationToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->trackingRailwayToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator2 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->saveRailVectorToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->savePointsToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator1 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->exportOptionsToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->exportGWADToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->exitToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->editToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->viewToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->optionsToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->setBackgroundColorToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->setPointCloudColorToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->setParametersToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->helpToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->dataConversionToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->statusStrip1 = (gcnew System::Windows::Forms::StatusStrip());
			this->toolStripProgressBar1 = (gcnew System::Windows::Forms::ToolStripProgressBar());
			this->toolStripStatusLabel1 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->openLASFileDialog = (gcnew System::Windows::Forms::OpenFileDialog());
			this->openPTSFileDialog = (gcnew System::Windows::Forms::OpenFileDialog());
			this->savePTSFileDialog = (gcnew System::Windows::Forms::SaveFileDialog());
			this->saveCSVFileDialog = (gcnew System::Windows::Forms::SaveFileDialog());
			this->toolStrip1 = (gcnew System::Windows::Forms::ToolStrip());
			this->toolStripButton1 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton2 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton3 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton4 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton5 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton6 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton7 = (gcnew System::Windows::Forms::ToolStripButton());
			this->toolStripButton8 = (gcnew System::Windows::Forms::ToolStripButton());
			this->tabPage2 = (gcnew System::Windows::Forms::TabPage());
			this->filesTreeView = (gcnew System::Windows::Forms::TreeView());
			this->tabPage1 = (gcnew System::Windows::Forms::TabPage());
			this->layersTreeView = (gcnew System::Windows::Forms::TreeView());
			this->tabControl1 = (gcnew System::Windows::Forms::TabControl());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->layersContextMenuStrip = (gcnew System::Windows::Forms::ContextMenuStrip(this->components));
			this->renameToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveVectorToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveGWADFileDialog = (gcnew System::Windows::Forms::SaveFileDialog());
			this->saveFileDialog1 = (gcnew System::Windows::Forms::SaveFileDialog());
			this->saveRailVectorWithIndexToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->menuStrip1->SuspendLayout();
			this->statusStrip1->SuspendLayout();
			this->toolStrip1->SuspendLayout();
			this->tabPage2->SuspendLayout();
			this->tabPage1->SuspendLayout();
			this->tabControl1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
			this->layersContextMenuStrip->SuspendLayout();
			this->SuspendLayout();
			// 
			// panel1
			// 
			this->panel1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->panel1->Location = System::Drawing::Point(179, 49);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(405, 291);
			this->panel1->TabIndex = 0;
			// 
			// menuStrip1
			// 
			this->menuStrip1->ImageScalingSize = System::Drawing::Size(24, 24);
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {
				this->fileToolStripMenuItem,
					this->editToolStripMenuItem, this->viewToolStripMenuItem, this->optionsToolStripMenuItem, this->helpToolStripMenuItem, this->dataConversionToolStripMenuItem
			});
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Padding = System::Windows::Forms::Padding(4, 2, 0, 2);
			this->menuStrip1->Size = System::Drawing::Size(584, 24);
			this->menuStrip1->TabIndex = 1;
			this->menuStrip1->Text = L"menuStrip1";
			// 
			// fileToolStripMenuItem
			// 
			this->fileToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(13) {
				this->openToolStripMenuItem,
					this->toolStripSeparator, this->selectInitialLocationToolStripMenuItem, this->setInitalLocationToolStripMenuItem, this->trackingRailwayToolStripMenuItem,
					this->toolStripSeparator2, this->saveRailVectorToolStripMenuItem, this->saveRailVectorWithIndexToolStripMenuItem, this->savePointsToolStripMenuItem,
					this->toolStripSeparator1, this->exportOptionsToolStripMenuItem, this->exportGWADToolStripMenuItem, this->exitToolStripMenuItem
			});
			this->fileToolStripMenuItem->Name = L"fileToolStripMenuItem";
			this->fileToolStripMenuItem->Size = System::Drawing::Size(37, 20);
			this->fileToolStripMenuItem->Text = L"&File";
			// 
			// openToolStripMenuItem
			// 
			this->openToolStripMenuItem->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"openToolStripMenuItem.Image")));
			this->openToolStripMenuItem->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->openToolStripMenuItem->Name = L"openToolStripMenuItem";
			this->openToolStripMenuItem->ShortcutKeys = static_cast<System::Windows::Forms::Keys>((System::Windows::Forms::Keys::Control | System::Windows::Forms::Keys::O));
			this->openToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->openToolStripMenuItem->Text = L"&Open";
			this->openToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::openToolStripMenuItem_Click);
			// 
			// toolStripSeparator
			// 
			this->toolStripSeparator->Name = L"toolStripSeparator";
			this->toolStripSeparator->Size = System::Drawing::Size(221, 6);
			// 
			// selectInitialLocationToolStripMenuItem
			// 
			this->selectInitialLocationToolStripMenuItem->Name = L"selectInitialLocationToolStripMenuItem";
			this->selectInitialLocationToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->selectInitialLocationToolStripMenuItem->Text = L"Select Initial Location";
			this->selectInitialLocationToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::selectInitialLocationToolStripMenuItem_Click);
			// 
			// setInitalLocationToolStripMenuItem
			// 
			this->setInitalLocationToolStripMenuItem->Name = L"setInitalLocationToolStripMenuItem";
			this->setInitalLocationToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->setInitalLocationToolStripMenuItem->Text = L"Set Initial Location";
			this->setInitalLocationToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::setInitalLocationToolStripMenuItem_Click);
			// 
			// trackingRailwayToolStripMenuItem
			// 
			this->trackingRailwayToolStripMenuItem->Name = L"trackingRailwayToolStripMenuItem";
			this->trackingRailwayToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->trackingRailwayToolStripMenuItem->Text = L"Tracking Railway";
			this->trackingRailwayToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::railwayTrackingToolStripMenuItem_Click);
			// 
			// toolStripSeparator2
			// 
			this->toolStripSeparator2->Name = L"toolStripSeparator2";
			this->toolStripSeparator2->Size = System::Drawing::Size(221, 6);
			// 
			// saveRailVectorToolStripMenuItem
			// 
			this->saveRailVectorToolStripMenuItem->Name = L"saveRailVectorToolStripMenuItem";
			this->saveRailVectorToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->saveRailVectorToolStripMenuItem->Text = L"Save Rail Vector";
			this->saveRailVectorToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::saveRailVectorToolStripMenuItem_Click);
			// 
			// savePointsToolStripMenuItem
			// 
			this->savePointsToolStripMenuItem->Name = L"savePointsToolStripMenuItem";
			this->savePointsToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->savePointsToolStripMenuItem->Text = L"Save Points";
			this->savePointsToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::savePointsToolStripMenuItem_Click);
			// 
			// toolStripSeparator1
			// 
			this->toolStripSeparator1->Name = L"toolStripSeparator1";
			this->toolStripSeparator1->Size = System::Drawing::Size(221, 6);
			// 
			// exportOptionsToolStripMenuItem
			// 
			this->exportOptionsToolStripMenuItem->Name = L"exportOptionsToolStripMenuItem";
			this->exportOptionsToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->exportOptionsToolStripMenuItem->Text = L"Export Options";
			this->exportOptionsToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::exportOptionsToolStripMenuItem_Click);
			// 
			// exportGWADToolStripMenuItem
			// 
			this->exportGWADToolStripMenuItem->Name = L"exportGWADToolStripMenuItem";
			this->exportGWADToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->exportGWADToolStripMenuItem->Text = L"Export GWAD";
			this->exportGWADToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::exportGWADToolStripMenuItem_Click);
			// 
			// exitToolStripMenuItem
			// 
			this->exitToolStripMenuItem->Name = L"exitToolStripMenuItem";
			this->exitToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->exitToolStripMenuItem->Text = L"E&xit";
			this->exitToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::exitToolStripMenuItem_Click);
			// 
			// editToolStripMenuItem
			// 
			this->editToolStripMenuItem->Enabled = false;
			this->editToolStripMenuItem->Name = L"editToolStripMenuItem";
			this->editToolStripMenuItem->Size = System::Drawing::Size(39, 20);
			this->editToolStripMenuItem->Text = L"Edit";
			// 
			// viewToolStripMenuItem
			// 
			this->viewToolStripMenuItem->Enabled = false;
			this->viewToolStripMenuItem->Name = L"viewToolStripMenuItem";
			this->viewToolStripMenuItem->Size = System::Drawing::Size(44, 20);
			this->viewToolStripMenuItem->Text = L"View";
			// 
			// optionsToolStripMenuItem
			// 
			this->optionsToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {
				this->setBackgroundColorToolStripMenuItem,
					this->setPointCloudColorToolStripMenuItem, this->setParametersToolStripMenuItem
			});
			this->optionsToolStripMenuItem->Name = L"optionsToolStripMenuItem";
			this->optionsToolStripMenuItem->Size = System::Drawing::Size(61, 20);
			this->optionsToolStripMenuItem->Text = L"Options";
			// 
			// setBackgroundColorToolStripMenuItem
			// 
			this->setBackgroundColorToolStripMenuItem->Name = L"setBackgroundColorToolStripMenuItem";
			this->setBackgroundColorToolStripMenuItem->Size = System::Drawing::Size(189, 22);
			this->setBackgroundColorToolStripMenuItem->Text = L"Set Background Color";
			this->setBackgroundColorToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::setBackgroundColorToolStripMenuItem_Click);
			// 
			// setPointCloudColorToolStripMenuItem
			// 
			this->setPointCloudColorToolStripMenuItem->Name = L"setPointCloudColorToolStripMenuItem";
			this->setPointCloudColorToolStripMenuItem->Size = System::Drawing::Size(189, 22);
			this->setPointCloudColorToolStripMenuItem->Text = L"Set Point Cloud Color";
			this->setPointCloudColorToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::setPointCloudColorToolStripMenuItem_Click);
			// 
			// setParametersToolStripMenuItem
			// 
			this->setParametersToolStripMenuItem->Name = L"setParametersToolStripMenuItem";
			this->setParametersToolStripMenuItem->Size = System::Drawing::Size(189, 22);
			this->setParametersToolStripMenuItem->Text = L"Set Parameters";
			this->setParametersToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::setParametersToolStripMenuItem_Click);
			// 
			// helpToolStripMenuItem
			// 
			this->helpToolStripMenuItem->Enabled = false;
			this->helpToolStripMenuItem->Name = L"helpToolStripMenuItem";
			this->helpToolStripMenuItem->Size = System::Drawing::Size(44, 20);
			this->helpToolStripMenuItem->Text = L"Help";
			// 
			// dataConversionToolStripMenuItem
			// 
			this->dataConversionToolStripMenuItem->Name = L"dataConversionToolStripMenuItem";
			this->dataConversionToolStripMenuItem->Size = System::Drawing::Size(106, 20);
			this->dataConversionToolStripMenuItem->Text = L"Data Conversion";
			this->dataConversionToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::dataConversionToolStripMenuItem_Click);
			// 
			// statusStrip1
			// 
			this->statusStrip1->ImageScalingSize = System::Drawing::Size(24, 24);
			this->statusStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
				this->toolStripProgressBar1,
					this->toolStripStatusLabel1
			});
			this->statusStrip1->Location = System::Drawing::Point(0, 340);
			this->statusStrip1->Name = L"statusStrip1";
			this->statusStrip1->Size = System::Drawing::Size(584, 22);
			this->statusStrip1->TabIndex = 2;
			this->statusStrip1->Text = L"statusStrip1";
			// 
			// toolStripProgressBar1
			// 
			this->toolStripProgressBar1->Name = L"toolStripProgressBar1";
			this->toolStripProgressBar1->Size = System::Drawing::Size(100, 16);
			// 
			// toolStripStatusLabel1
			// 
			this->toolStripStatusLabel1->Name = L"toolStripStatusLabel1";
			this->toolStripStatusLabel1->Size = System::Drawing::Size(64, 17);
			this->toolStripStatusLabel1->Text = L"Status: Idle";
			this->toolStripStatusLabel1->Click += gcnew System::EventHandler(this, &MainForm::toolStripStatusLabel1_Click);
			// 
			// openLASFileDialog
			// 
			this->openLASFileDialog->DefaultExt = L"las";
			this->openLASFileDialog->Filter = L"LAS files (*.las)|*.las|LAZ files (*.laz)|*.laz|PTS files (*.pts)|*.pts";
			this->openLASFileDialog->Multiselect = true;
			// 
			// openPTSFileDialog
			// 
			this->openPTSFileDialog->DefaultExt = L"pts";
			this->openPTSFileDialog->Filter = L"PTS files (*.pts)|*.pts";
			this->openPTSFileDialog->Multiselect = true;
			this->openPTSFileDialog->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &MainForm::OpenPTSFileDialog_FileOk);
			// 
			// savePTSFileDialog
			// 
			this->savePTSFileDialog->DefaultExt = L"pts";
			this->savePTSFileDialog->Filter = L"PTS files (*.pts)|*.pts";
			// 
			// saveCSVFileDialog
			// 
			this->saveCSVFileDialog->DefaultExt = L"csv";
			this->saveCSVFileDialog->Filter = L"CSV files (*.csv)|*.csv";
			// 
			// toolStrip1
			// 
			this->toolStrip1->ImageScalingSize = System::Drawing::Size(24, 24);
			this->toolStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(8) {
				this->toolStripButton1,
					this->toolStripButton2, this->toolStripButton3, this->toolStripButton4, this->toolStripButton5, this->toolStripButton6, this->toolStripButton7,
					this->toolStripButton8
			});
			this->toolStrip1->Location = System::Drawing::Point(0, 24);
			this->toolStrip1->Name = L"toolStrip1";
			this->toolStrip1->Size = System::Drawing::Size(584, 31);
			this->toolStrip1->TabIndex = 3;
			this->toolStrip1->Text = L"toolStrip1";
			// 
			// toolStripButton1
			// 
			this->toolStripButton1->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton1.Image")));
			this->toolStripButton1->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton1->Name = L"toolStripButton1";
			this->toolStripButton1->Size = System::Drawing::Size(28, 28);
			this->toolStripButton1->Text = L"toolStripButton1";
			this->toolStripButton1->ToolTipText = L"Isometric";
			this->toolStripButton1->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraIsometric_Click);
			// 
			// toolStripButton2
			// 
			this->toolStripButton2->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton2->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton2.Image")));
			this->toolStripButton2->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton2->Name = L"toolStripButton2";
			this->toolStripButton2->Size = System::Drawing::Size(28, 28);
			this->toolStripButton2->Text = L"toolStripButton2";
			this->toolStripButton2->ToolTipText = L"Front";
			this->toolStripButton2->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraFront_Click);
			// 
			// toolStripButton3
			// 
			this->toolStripButton3->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton3->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton3.Image")));
			this->toolStripButton3->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton3->Name = L"toolStripButton3";
			this->toolStripButton3->Size = System::Drawing::Size(28, 28);
			this->toolStripButton3->Text = L"toolStripButton3";
			this->toolStripButton3->ToolTipText = L"Back";
			this->toolStripButton3->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraBack_Click);
			// 
			// toolStripButton4
			// 
			this->toolStripButton4->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton4->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton4.Image")));
			this->toolStripButton4->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton4->Name = L"toolStripButton4";
			this->toolStripButton4->Size = System::Drawing::Size(28, 28);
			this->toolStripButton4->Text = L"toolStripButton4";
			this->toolStripButton4->ToolTipText = L"Top";
			this->toolStripButton4->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraTop_Click);
			// 
			// toolStripButton5
			// 
			this->toolStripButton5->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton5->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton5.Image")));
			this->toolStripButton5->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton5->Name = L"toolStripButton5";
			this->toolStripButton5->Size = System::Drawing::Size(28, 28);
			this->toolStripButton5->Text = L"toolStripButton5";
			this->toolStripButton5->ToolTipText = L"Bottom";
			this->toolStripButton5->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraDown_Click);
			// 
			// toolStripButton6
			// 
			this->toolStripButton6->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton6->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton6.Image")));
			this->toolStripButton6->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton6->Name = L"toolStripButton6";
			this->toolStripButton6->Size = System::Drawing::Size(28, 28);
			this->toolStripButton6->Text = L"toolStripButton6";
			this->toolStripButton6->ToolTipText = L"Left";
			this->toolStripButton6->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraLeft_Click);
			// 
			// toolStripButton7
			// 
			this->toolStripButton7->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton7->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton7.Image")));
			this->toolStripButton7->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton7->Name = L"toolStripButton7";
			this->toolStripButton7->Size = System::Drawing::Size(28, 28);
			this->toolStripButton7->Text = L"toolStripButton7";
			this->toolStripButton7->ToolTipText = L"Right";
			this->toolStripButton7->Click += gcnew System::EventHandler(this, &MainForm::toolStripCameraRight_Click);
			// 
			// toolStripButton8
			// 
			this->toolStripButton8->DisplayStyle = System::Windows::Forms::ToolStripItemDisplayStyle::Image;
			this->toolStripButton8->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"toolStripButton8.Image")));
			this->toolStripButton8->ImageTransparentColor = System::Drawing::Color::Magenta;
			this->toolStripButton8->Name = L"toolStripButton8";
			this->toolStripButton8->Size = System::Drawing::Size(28, 28);
			this->toolStripButton8->Text = L"toolStripButton8";
			this->toolStripButton8->Click += gcnew System::EventHandler(this, &MainForm::toolIntensity_Click);
			// 
			// tabPage2
			// 
			this->tabPage2->Controls->Add(this->filesTreeView);
			this->tabPage2->Location = System::Drawing::Point(4, 22);
			this->tabPage2->Name = L"tabPage2";
			this->tabPage2->Padding = System::Windows::Forms::Padding(3, 3, 3, 3);
			this->tabPage2->Size = System::Drawing::Size(173, 265);
			this->tabPage2->TabIndex = 1;
			this->tabPage2->Text = L"Files";
			this->tabPage2->UseVisualStyleBackColor = true;
			// 
			// filesTreeView
			// 
			this->filesTreeView->Dock = System::Windows::Forms::DockStyle::Fill;
			this->filesTreeView->Location = System::Drawing::Point(3, 3);
			this->filesTreeView->Name = L"filesTreeView";
			treeNode1->Name = L"Node1";
			treeNode1->Text = L"file1.las";
			treeNode2->Name = L"Node2";
			treeNode2->Text = L"file2.pts";
			treeNode3->Name = L"Node3";
			treeNode3->Text = L"file3.laz";
			treeNode4->Name = L"Node0";
			treeNode4->Text = L"Project 1";
			this->filesTreeView->Nodes->AddRange(gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) { treeNode4 });
			this->filesTreeView->Size = System::Drawing::Size(167, 259);
			this->filesTreeView->TabIndex = 0;
			this->filesTreeView->NodeMouseDoubleClick += gcnew System::Windows::Forms::TreeNodeMouseClickEventHandler(this, &MainForm::filesTreeView_NodeMouseDoubleClick);
			// 
			// tabPage1
			// 
			this->tabPage1->Controls->Add(this->layersTreeView);
			this->tabPage1->Location = System::Drawing::Point(4, 22);
			this->tabPage1->Name = L"tabPage1";
			this->tabPage1->Padding = System::Windows::Forms::Padding(3, 3, 3, 3);
			this->tabPage1->Size = System::Drawing::Size(173, 265);
			this->tabPage1->TabIndex = 0;
			this->tabPage1->Text = L"Layers";
			this->tabPage1->UseVisualStyleBackColor = true;
			// 
			// layersTreeView
			// 
			this->layersTreeView->CheckBoxes = true;
			this->layersTreeView->Dock = System::Windows::Forms::DockStyle::Fill;
			this->layersTreeView->LabelEdit = true;
			this->layersTreeView->Location = System::Drawing::Point(3, 3);
			this->layersTreeView->Name = L"layersTreeView";
			treeNode5->Name = L"Node4";
			treeNode5->Text = L"Node4";
			treeNode6->Name = L"Node2";
			treeNode6->Text = L"Node2";
			treeNode7->Name = L"Node0";
			treeNode7->Text = L"Node0";
			treeNode8->Name = L"Node5";
			treeNode8->Text = L"Node5";
			treeNode9->Name = L"Node3";
			treeNode9->Text = L"Node3";
			treeNode10->Name = L"Node1";
			treeNode10->Text = L"Node1";
			this->layersTreeView->Nodes->AddRange(gcnew cli::array< System::Windows::Forms::TreeNode^  >(2) { treeNode7, treeNode10 });
			this->layersTreeView->Size = System::Drawing::Size(167, 259);
			this->layersTreeView->TabIndex = 1;
			this->layersTreeView->AfterCheck += gcnew System::Windows::Forms::TreeViewEventHandler(this, &MainForm::layersTreeView_AfterCheck);
			this->layersTreeView->NodeMouseClick += gcnew System::Windows::Forms::TreeNodeMouseClickEventHandler(this, &MainForm::layersTreeView_NodeMouseClick);
			this->layersTreeView->EnabledChanged += gcnew System::EventHandler(this, &MainForm::layersTreeView_EnabledChanged);
			// 
			// tabControl1
			// 
			this->tabControl1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>(((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left));
			this->tabControl1->Controls->Add(this->tabPage1);
			this->tabControl1->Controls->Add(this->tabPage2);
			this->tabControl1->Location = System::Drawing::Point(0, 49);
			this->tabControl1->Name = L"tabControl1";
			this->tabControl1->SelectedIndex = 0;
			this->tabControl1->Size = System::Drawing::Size(181, 291);
			this->tabControl1->TabIndex = 4;
			// 
			// trackBar1
			// 
			this->trackBar1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->trackBar1->Location = System::Drawing::Point(459, 340);
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->RightToLeft = System::Windows::Forms::RightToLeft::No;
			this->trackBar1->Size = System::Drawing::Size(104, 45);
			this->trackBar1->TabIndex = 5;
			this->trackBar1->TickStyle = System::Windows::Forms::TickStyle::None;
			// 
			// layersContextMenuStrip
			// 
			this->layersContextMenuStrip->ImageScalingSize = System::Drawing::Size(24, 24);
			this->layersContextMenuStrip->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
				this->renameToolStripMenuItem,
					this->saveVectorToolStripMenuItem
			});
			this->layersContextMenuStrip->Name = L"contextMenuStrip1";
			this->layersContextMenuStrip->Size = System::Drawing::Size(157, 48);
			this->layersContextMenuStrip->Opening += gcnew System::ComponentModel::CancelEventHandler(this, &MainForm::LayersContextMenuStrip_Opening);
			// 
			// renameToolStripMenuItem
			// 
			this->renameToolStripMenuItem->Name = L"renameToolStripMenuItem";
			this->renameToolStripMenuItem->Size = System::Drawing::Size(156, 22);
			this->renameToolStripMenuItem->Text = L"Rename";
			this->renameToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::renameToolStripMenuItem_Click);
			// 
			// saveVectorToolStripMenuItem
			// 
			this->saveVectorToolStripMenuItem->Name = L"saveVectorToolStripMenuItem";
			this->saveVectorToolStripMenuItem->Size = System::Drawing::Size(156, 22);
			this->saveVectorToolStripMenuItem->Text = L"Save Rail Vector";
			this->saveVectorToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::SaveVectorToolStripMenuItem_Click);
			// 
			// saveGWADFileDialog
			// 
			this->saveGWADFileDialog->Filter = L"GWAD files (*.gwad)|*.gwad";
			// 
			// saveRailVectorWithIndexToolStripMenuItem
			// 
			this->saveRailVectorWithIndexToolStripMenuItem->Name = L"saveRailVectorWithIndexToolStripMenuItem";
			this->saveRailVectorWithIndexToolStripMenuItem->Size = System::Drawing::Size(224, 30);
			this->saveRailVectorWithIndexToolStripMenuItem->Text = L"Save Rail Vector With Index";
			this->saveRailVectorWithIndexToolStripMenuItem->Click += gcnew System::EventHandler(this, &MainForm::saveRailVectorWithIndexToolStripMenuItem_Click);
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(584, 362);
			this->Controls->Add(this->trackBar1);
			this->Controls->Add(this->tabControl1);
			this->Controls->Add(this->panel1);
			this->Controls->Add(this->toolStrip1);
			this->Controls->Add(this->statusStrip1);
			this->Controls->Add(this->menuStrip1);
			this->MainMenuStrip = this->menuStrip1;
			this->Name = L"MainForm";
			this->Text = L"Viewer";
			this->WindowState = System::Windows::Forms::FormWindowState::Maximized;
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			this->statusStrip1->ResumeLayout(false);
			this->statusStrip1->PerformLayout();
			this->toolStrip1->ResumeLayout(false);
			this->toolStrip1->PerformLayout();
			this->tabPage2->ResumeLayout(false);
			this->tabPage1->ResumeLayout(false);
			this->tabControl1->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
			this->layersContextMenuStrip->ResumeLayout(false);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

		private: VTKWindowControl ^vtkWindowControl;
		private: LASFileReader *reader;
		private: TXTFileReader *txtReader;

		private: RailwayExtraction *railwayExtraction;
		private: DataConvertor *convertor;

		private: void AddCustomControls()
		{
			this->vtkWindowControl = (gcnew VTKWindowControl());

			this->vtkWindowControl->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->vtkWindowControl->Location = System::Drawing::Point(0, 0);
			this->vtkWindowControl->Name = L"VTKWindowControl";
			this->vtkWindowControl->Dock = System::Windows::Forms::DockStyle::Fill;
			this->vtkWindowControl->Size = System::Drawing::Size(284, 262);
			this->vtkWindowControl->mousePositionLabel = this->toolStripStatusLabel1;
			//this->vtkWindowControl->mainForm = this;

			this->panel1->Controls->Add(vtkWindowControl);
		}

		private: System::Void openToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void setInitalLocationToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void railwayTrackingToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void saveRailVectorToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void savePointsToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void exitToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);

		private: System::Void showPointCloud(pcl::PointCloud<PointXYZDouble>::Ptr point_cloud);
		
		private: System::Void toolStripStatusLabel1_Click(System::Object^  sender, System::EventArgs^  e) {
		}

		private: System::Void toolStripCameraTop_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraDown_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraLeft_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraRight_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraFront_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraBack_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolStripCameraIsometric_Click(System::Object^  sender, System::EventArgs^  e);

		public: System::Void VTKMouseMove(int x, int y, int z);
		public: System::Void GenerateGrid();
		private: System::Void saveRailVectorAtIndex(int index);
		private: System::Void saveRailVectorWithIndexAtIndex(int index);


		public: static System::Void LoadPointCloud(System::Object ^obj);
		private: System::Void InitLocationPick();

		delegate void ManagedPointsCallback(double x1, double y1, double z1, double x2, double y2, double z2);
		public: void ReceivedPoints(double x1, double y1, double z1, double x2, double y2, double z2);

		private: double GetAngle(double x1, double y1, double x2, double y2);
		
		private: System::Void setBackgroundColorToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void toolIntensity_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void layersTreeView_EnabledChanged(System::Object^  sender, System::EventArgs^  e);
		private: System::Void layersTreeView_AfterCheck(System::Object^  sender, System::Windows::Forms::TreeViewEventArgs^  e);
		private: System::Void renameToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void layersTreeView_NodeMouseClick(System::Object^  sender, System::Windows::Forms::TreeNodeMouseClickEventArgs^  e);


		//Private members
		private: TreeNode ^selectedNode;
		private: System::Void openPTSFileToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void eCEFLASToUTMToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void uTMPTSToECEFPTSToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);

		private: void PCLProject::MainForm::UpdateProgress(double readPoints, double totalPoints);
		private: System::Void dataConversionToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void selectInitialLocationToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);

		private: System::Void listFiles(cli::array<String ^> ^fileNames);
		private: System::Void loadFile(String ^fileName);
		private: System::Void updateTrackedLayers();

		private: System::Void filesTreeView_NodeMouseDoubleClick(System::Object^  sender, System::Windows::Forms::TreeNodeMouseClickEventArgs^  e);
		private: System::Void exportGWADToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void setParametersToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void computeGradeToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void setPointCloudColorToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
		private: System::Void exportOptionsToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void OpenPTSFileDialog_FileOk(System::Object^ sender, System::ComponentModel::CancelEventArgs^ e) {
}
private: System::Void LayersContextMenuStrip_Opening(System::Object^ sender, System::ComponentModel::CancelEventArgs^ e) {
}
private: System::Void SaveVectorToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e);
private: System::Void saveRailVectorWithIndexToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e);
};
}
