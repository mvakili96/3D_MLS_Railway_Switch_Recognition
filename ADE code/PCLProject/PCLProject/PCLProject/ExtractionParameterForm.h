#pragma once

#include "LIDARPT.h"

namespace PCLProject {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for ExtractionParameterForm
	/// </summary>
	public ref class ExtractionParameterForm : public System::Windows::Forms::Form
	{
	public:
		ExtractionParameterForm(void)
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
		~ExtractionParameterForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::TextBox^  trackWindowLenghtTextBox;
	private: System::Windows::Forms::TextBox^  trackWindowWidthTextBox;
	private: System::Windows::Forms::TextBox^  TrackGuageTextBox;
	private: System::Windows::Forms::TextBox^  minTrackPointsTextBox;
	private: System::Windows::Forms::TextBox^  minPointsTextBox;
	private: System::Windows::Forms::TextBox^  maxDistanceTextBox;
	private: System::Windows::Forms::TextBox^  distanceGroupingTextBox;
	private: System::Windows::Forms::TextBox^  weightParameterTextBox;
	protected:








	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::TextBox^  minIntensityRangeTextBox;
	private: System::Windows::Forms::TextBox^  maxIntensityRangeTextBox;
	private: System::Windows::Forms::Button^  okayButton;
	private: System::Windows::Forms::Button^  cancelButton;


	protected:

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
			this->trackWindowLenghtTextBox = (gcnew System::Windows::Forms::TextBox());
			this->trackWindowWidthTextBox = (gcnew System::Windows::Forms::TextBox());
			this->TrackGuageTextBox = (gcnew System::Windows::Forms::TextBox());
			this->minTrackPointsTextBox = (gcnew System::Windows::Forms::TextBox());
			this->minPointsTextBox = (gcnew System::Windows::Forms::TextBox());
			this->maxDistanceTextBox = (gcnew System::Windows::Forms::TextBox());
			this->distanceGroupingTextBox = (gcnew System::Windows::Forms::TextBox());
			this->weightParameterTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->minIntensityRangeTextBox = (gcnew System::Windows::Forms::TextBox());
			this->maxIntensityRangeTextBox = (gcnew System::Windows::Forms::TextBox());
			this->okayButton = (gcnew System::Windows::Forms::Button());
			this->cancelButton = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// trackWindowLenghtTextBox
			// 
			this->trackWindowLenghtTextBox->Location = System::Drawing::Point(192, 22);
			this->trackWindowLenghtTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->trackWindowLenghtTextBox->Name = L"trackWindowLenghtTextBox";
			this->trackWindowLenghtTextBox->Size = System::Drawing::Size(251, 22);
			this->trackWindowLenghtTextBox->TabIndex = 0;
			// 
			// trackWindowWidthTextBox
			// 
			this->trackWindowWidthTextBox->Location = System::Drawing::Point(192, 56);
			this->trackWindowWidthTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->trackWindowWidthTextBox->Name = L"trackWindowWidthTextBox";
			this->trackWindowWidthTextBox->Size = System::Drawing::Size(251, 22);
			this->trackWindowWidthTextBox->TabIndex = 1;
			// 
			// TrackGuageTextBox
			// 
			this->TrackGuageTextBox->Location = System::Drawing::Point(192, 94);
			this->TrackGuageTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->TrackGuageTextBox->Name = L"TrackGuageTextBox";
			this->TrackGuageTextBox->Size = System::Drawing::Size(251, 22);
			this->TrackGuageTextBox->TabIndex = 2;
			// 
			// minTrackPointsTextBox
			// 
			this->minTrackPointsTextBox->Location = System::Drawing::Point(192, 136);
			this->minTrackPointsTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->minTrackPointsTextBox->Name = L"minTrackPointsTextBox";
			this->minTrackPointsTextBox->Size = System::Drawing::Size(251, 22);
			this->minTrackPointsTextBox->TabIndex = 3;
			// 
			// minPointsTextBox
			// 
			this->minPointsTextBox->Location = System::Drawing::Point(192, 174);
			this->minPointsTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->minPointsTextBox->Name = L"minPointsTextBox";
			this->minPointsTextBox->Size = System::Drawing::Size(251, 22);
			this->minPointsTextBox->TabIndex = 4;
			// 
			// maxDistanceTextBox
			// 
			this->maxDistanceTextBox->Location = System::Drawing::Point(192, 211);
			this->maxDistanceTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->maxDistanceTextBox->Name = L"maxDistanceTextBox";
			this->maxDistanceTextBox->Size = System::Drawing::Size(251, 22);
			this->maxDistanceTextBox->TabIndex = 5;
			// 
			// distanceGroupingTextBox
			// 
			this->distanceGroupingTextBox->Location = System::Drawing::Point(192, 250);
			this->distanceGroupingTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->distanceGroupingTextBox->Name = L"distanceGroupingTextBox";
			this->distanceGroupingTextBox->Size = System::Drawing::Size(251, 22);
			this->distanceGroupingTextBox->TabIndex = 6;
			// 
			// weightParameterTextBox
			// 
			this->weightParameterTextBox->Location = System::Drawing::Point(192, 290);
			this->weightParameterTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->weightParameterTextBox->Name = L"weightParameterTextBox";
			this->weightParameterTextBox->Size = System::Drawing::Size(251, 22);
			this->weightParameterTextBox->TabIndex = 7;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(12, 24);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(149, 17);
			this->label1->TabIndex = 8;
			this->label1->Text = L"Track Window Length:";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(11, 61);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(141, 17);
			this->label2->TabIndex = 9;
			this->label2->Text = L"Track Window Width:";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(11, 98);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(95, 17);
			this->label3->TabIndex = 10;
			this->label3->Text = L"Track Guage:";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(12, 141);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(177, 17);
			this->label4->TabIndex = 11;
			this->label4->Text = L"Minimum # of Track points:";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(12, 177);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(137, 17);
			this->label5->TabIndex = 12;
			this->label5->Text = L"Minimum # of points:";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(13, 214);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(172, 17);
			this->label6->TabIndex = 13;
			this->label6->Text = L"Max Distance of RANSAC:";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(13, 253);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(146, 17);
			this->label7->TabIndex = 14;
			this->label7->Text = L"Distance of Grouping:";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(13, 290);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(174, 17);
			this->label8->TabIndex = 15;
			this->label8->Text = L"Weight parameter of MDL:";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(13, 326);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(166, 17);
			this->label9->TabIndex = 16;
			this->label9->Text = L"Intensity Range of Track:";
			// 
			// minIntensityRangeTextBox
			// 
			this->minIntensityRangeTextBox->Location = System::Drawing::Point(192, 326);
			this->minIntensityRangeTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->minIntensityRangeTextBox->Name = L"minIntensityRangeTextBox";
			this->minIntensityRangeTextBox->Size = System::Drawing::Size(126, 22);
			this->minIntensityRangeTextBox->TabIndex = 17;
			// 
			// maxIntensityRangeTextBox
			// 
			this->maxIntensityRangeTextBox->Location = System::Drawing::Point(323, 326);
			this->maxIntensityRangeTextBox->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->maxIntensityRangeTextBox->Name = L"maxIntensityRangeTextBox";
			this->maxIntensityRangeTextBox->Size = System::Drawing::Size(120, 22);
			this->maxIntensityRangeTextBox->TabIndex = 18;
			// 
			// okayButton
			// 
			this->okayButton->Location = System::Drawing::Point(14, 366);
			this->okayButton->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->okayButton->Name = L"okayButton";
			this->okayButton->Size = System::Drawing::Size(198, 30);
			this->okayButton->TabIndex = 19;
			this->okayButton->Text = L"OK";
			this->okayButton->UseVisualStyleBackColor = true;
			this->okayButton->Click += gcnew System::EventHandler(this, &ExtractionParameterForm::okayButton_Click);
			// 
			// cancelButton
			// 
			this->cancelButton->Location = System::Drawing::Point(244, 366);
			this->cancelButton->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->cancelButton->Name = L"cancelButton";
			this->cancelButton->Size = System::Drawing::Size(198, 30);
			this->cancelButton->TabIndex = 20;
			this->cancelButton->Text = L"Cancel";
			this->cancelButton->UseVisualStyleBackColor = true;
			this->cancelButton->Click += gcnew System::EventHandler(this, &ExtractionParameterForm::cancelButton_Click);
			// 
			// ExtractionParameterForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(453, 411);
			this->Controls->Add(this->cancelButton);
			this->Controls->Add(this->okayButton);
			this->Controls->Add(this->maxIntensityRangeTextBox);
			this->Controls->Add(this->minIntensityRangeTextBox);
			this->Controls->Add(this->label9);
			this->Controls->Add(this->label8);
			this->Controls->Add(this->label7);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->weightParameterTextBox);
			this->Controls->Add(this->distanceGroupingTextBox);
			this->Controls->Add(this->maxDistanceTextBox);
			this->Controls->Add(this->minPointsTextBox);
			this->Controls->Add(this->minTrackPointsTextBox);
			this->Controls->Add(this->TrackGuageTextBox);
			this->Controls->Add(this->trackWindowWidthTextBox);
			this->Controls->Add(this->trackWindowLenghtTextBox);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedDialog;
			this->Margin = System::Windows::Forms::Padding(3, 2, 3, 2);
			this->MaximizeBox = false;
			this->Name = L"ExtractionParameterForm";
			this->Text = L"Extraction Algorithm Parameters";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void okayButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void cancelButton_Click(System::Object^  sender, System::EventArgs^  e);

	public: ExtractionParameters getValues();
	public: void updateValues(ExtractionParameters parameters);
};
}
