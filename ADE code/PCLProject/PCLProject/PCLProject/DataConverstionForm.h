#pragma once
#include "DataConvertor.h"
#include <array>

namespace PCLProject {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for DataConverstionForm
	/// </summary>
	public ref class DataConverstionForm : public System::Windows::Forms::Form
	{
	public:
		DataConverstionForm(void)
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
		~DataConverstionForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::TextBox^  sourceFilesFolderTextBox;
	protected:

	protected:
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Button^  sourceButton;
	private: System::Windows::Forms::Button^  destinationButton;


	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  destinationFolderTextBox;

	private: System::Windows::Forms::OpenFileDialog^  openFileDialog;
	private: System::Windows::Forms::FolderBrowserDialog^  folderBrowserDialog;
	private: System::Windows::Forms::ProgressBar^  fileProgressBar;
	private: System::Windows::Forms::ProgressBar^  allFilesProgressBar;


	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  fileProgressLabel;
	private: System::Windows::Forms::Button^  ecefConvertButton;
	private: System::Windows::Forms::Button^  utmConvertButton;
	private: System::Windows::Forms::TextBox^  zoneTextBox;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;







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
			this->sourceFilesFolderTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->sourceButton = (gcnew System::Windows::Forms::Button());
			this->destinationButton = (gcnew System::Windows::Forms::Button());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->destinationFolderTextBox = (gcnew System::Windows::Forms::TextBox());
			this->openFileDialog = (gcnew System::Windows::Forms::OpenFileDialog());
			this->folderBrowserDialog = (gcnew System::Windows::Forms::FolderBrowserDialog());
			this->fileProgressBar = (gcnew System::Windows::Forms::ProgressBar());
			this->allFilesProgressBar = (gcnew System::Windows::Forms::ProgressBar());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->fileProgressLabel = (gcnew System::Windows::Forms::Label());
			this->ecefConvertButton = (gcnew System::Windows::Forms::Button());
			this->utmConvertButton = (gcnew System::Windows::Forms::Button());
			this->zoneTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->SuspendLayout();
			// 
			// sourceFilesFolderTextBox
			// 
			this->sourceFilesFolderTextBox->Location = System::Drawing::Point(75, 8);
			this->sourceFilesFolderTextBox->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->sourceFilesFolderTextBox->Name = L"sourceFilesFolderTextBox";
			this->sourceFilesFolderTextBox->Size = System::Drawing::Size(413, 20);
			this->sourceFilesFolderTextBox->TabIndex = 0;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(27, 10);
			this->label1->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(44, 13);
			this->label1->TabIndex = 1;
			this->label1->Text = L"Source:";
			// 
			// sourceButton
			// 
			this->sourceButton->Location = System::Drawing::Point(490, 6);
			this->sourceButton->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->sourceButton->Name = L"sourceButton";
			this->sourceButton->Size = System::Drawing::Size(50, 21);
			this->sourceButton->TabIndex = 2;
			this->sourceButton->Text = L"Browse";
			this->sourceButton->UseVisualStyleBackColor = true;
			this->sourceButton->Click += gcnew System::EventHandler(this, &DataConverstionForm::sourceButton_Click);
			// 
			// destinationButton
			// 
			this->destinationButton->Location = System::Drawing::Point(490, 36);
			this->destinationButton->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->destinationButton->Name = L"destinationButton";
			this->destinationButton->Size = System::Drawing::Size(50, 21);
			this->destinationButton->TabIndex = 5;
			this->destinationButton->Text = L"Browse";
			this->destinationButton->UseVisualStyleBackColor = true;
			this->destinationButton->Click += gcnew System::EventHandler(this, &DataConverstionForm::destinationButton_Click);
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(8, 40);
			this->label2->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(63, 13);
			this->label2->TabIndex = 4;
			this->label2->Text = L"Destination:";
			// 
			// destinationFolderTextBox
			// 
			this->destinationFolderTextBox->Location = System::Drawing::Point(75, 38);
			this->destinationFolderTextBox->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->destinationFolderTextBox->Name = L"destinationFolderTextBox";
			this->destinationFolderTextBox->Size = System::Drawing::Size(413, 20);
			this->destinationFolderTextBox->TabIndex = 3;
			// 
			// openFileDialog
			// 
			this->openFileDialog->CheckFileExists = false;
			this->openFileDialog->Filter = L"LAS files (*.las)|*.las|PTS files (*.pts)|*.pts|CSV files (*.csv)|*.csv";
			this->openFileDialog->Multiselect = true;
			this->openFileDialog->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &DataConverstionForm::OpenFileDialog_FileOk);
			// 
			// fileProgressBar
			// 
			this->fileProgressBar->Location = System::Drawing::Point(11, 136);
			this->fileProgressBar->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->fileProgressBar->Name = L"fileProgressBar";
			this->fileProgressBar->Size = System::Drawing::Size(529, 15);
			this->fileProgressBar->TabIndex = 6;
			// 
			// allFilesProgressBar
			// 
			this->allFilesProgressBar->Location = System::Drawing::Point(11, 183);
			this->allFilesProgressBar->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->allFilesProgressBar->Name = L"allFilesProgressBar";
			this->allFilesProgressBar->Size = System::Drawing::Size(529, 15);
			this->allFilesProgressBar->TabIndex = 7;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(8, 166);
			this->label3->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(84, 13);
			this->label3->TabIndex = 8;
			this->label3->Text = L"Overall Progress";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(9, 119);
			this->label4->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(70, 13);
			this->label4->TabIndex = 9;
			this->label4->Text = L"File Progress:";
			// 
			// fileProgressLabel
			// 
			this->fileProgressLabel->AutoSize = true;
			this->fileProgressLabel->Location = System::Drawing::Point(75, 120);
			this->fileProgressLabel->Margin = System::Windows::Forms::Padding(2, 0, 2, 0);
			this->fileProgressLabel->Name = L"fileProgressLabel";
			this->fileProgressLabel->Size = System::Drawing::Size(30, 13);
			this->fileProgressLabel->TabIndex = 10;
			this->fileProgressLabel->Text = L"0 / 0";
			// 
			// ecefConvertButton
			// 
			this->ecefConvertButton->Enabled = false;
			this->ecefConvertButton->Location = System::Drawing::Point(397, 81);
			this->ecefConvertButton->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->ecefConvertButton->Name = L"ecefConvertButton";
			this->ecefConvertButton->Size = System::Drawing::Size(142, 21);
			this->ecefConvertButton->TabIndex = 11;
			this->ecefConvertButton->Text = L"UTM to ECEF";
			this->ecefConvertButton->UseVisualStyleBackColor = true;
			this->ecefConvertButton->Click += gcnew System::EventHandler(this, &DataConverstionForm::ecefConvertButton_Click);
			// 
			// utmConvertButton
			// 
			this->utmConvertButton->Enabled = false;
			this->utmConvertButton->Location = System::Drawing::Point(251, 81);
			this->utmConvertButton->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->utmConvertButton->Name = L"utmConvertButton";
			this->utmConvertButton->Size = System::Drawing::Size(142, 21);
			this->utmConvertButton->TabIndex = 12;
			this->utmConvertButton->Text = L"ECEF to UTM";
			this->utmConvertButton->UseVisualStyleBackColor = true;
			this->utmConvertButton->Click += gcnew System::EventHandler(this, &DataConverstionForm::utmConvertButton_Click);
			// 
			// zoneTextBox
			// 
			this->zoneTextBox->Location = System::Drawing::Point(75, 81);
			this->zoneTextBox->Name = L"zoneTextBox";
			this->zoneTextBox->Size = System::Drawing::Size(100, 20);
			this->zoneTextBox->TabIndex = 13;
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(11, 87);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(32, 13);
			this->label5->TabIndex = 14;
			this->label5->Text = L"Zone";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(120, 95);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(0, 13);
			this->label6->TabIndex = 15;
			// 
			// DataConverstionForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(548, 214);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->zoneTextBox);
			this->Controls->Add(this->utmConvertButton);
			this->Controls->Add(this->ecefConvertButton);
			this->Controls->Add(this->fileProgressLabel);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->allFilesProgressBar);
			this->Controls->Add(this->fileProgressBar);
			this->Controls->Add(this->destinationButton);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->destinationFolderTextBox);
			this->Controls->Add(this->sourceButton);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->sourceFilesFolderTextBox);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedDialog;
			this->Margin = System::Windows::Forms::Padding(2, 2, 2, 2);
			this->Name = L"DataConverstionForm";
			this->Text = L"Data Converstion";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void sourceButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void destinationButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void utmConvertButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void ecefConvertButton_Click(System::Object^  sender, System::EventArgs^  e);

	private: void UpdateProgress(double readPoints, double totalPoints);
	private: void UpdateAllFilesProgress(double readPoints, double totalPoints);
	private: bool CheckErrors();
private: System::Void OpenFileDialog_FileOk(System::Object^ sender, System::ComponentModel::CancelEventArgs^ e) {
}
};
}
