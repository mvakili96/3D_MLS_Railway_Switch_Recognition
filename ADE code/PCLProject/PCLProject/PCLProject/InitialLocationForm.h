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
	/// Summary for InitialLocationForm
	/// </summary>
	public ref class InitialLocationForm : public System::Windows::Forms::Form
	{
	public:
		InitialLocationForm(void)
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
		~InitialLocationForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Label^  label1;


	protected:

	private: System::Windows::Forms::TextBox^  xAxisTextBox;

	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  yAxisTextBox;

	private: System::Windows::Forms::Label^  label3;


	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Button^  okayButton;

	private: System::Windows::Forms::Button^  pickAgainButton;

	private: System::Windows::Forms::Button^  cancelButton;
	private: System::Windows::Forms::NumericUpDown^  angleUpDown;
	private: System::Windows::Forms::NumericUpDown^  modeUpDown;
	private: System::Windows::Forms::TextBox^  zAxisTextBox;
	private: System::Windows::Forms::Label^  label5;





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
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->xAxisTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->yAxisTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->okayButton = (gcnew System::Windows::Forms::Button());
			this->pickAgainButton = (gcnew System::Windows::Forms::Button());
			this->cancelButton = (gcnew System::Windows::Forms::Button());
			this->angleUpDown = (gcnew System::Windows::Forms::NumericUpDown());
			this->modeUpDown = (gcnew System::Windows::Forms::NumericUpDown());
			this->zAxisTextBox = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->angleUpDown))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->modeUpDown))->BeginInit();
			this->SuspendLayout();
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(11, 93);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(37, 13);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Angle:";
			// 
			// xAxisTextBox
			// 
			this->xAxisTextBox->Location = System::Drawing::Point(54, 12);
			this->xAxisTextBox->Name = L"xAxisTextBox";
			this->xAxisTextBox->Size = System::Drawing::Size(197, 20);
			this->xAxisTextBox->TabIndex = 1;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(31, 15);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(17, 13);
			this->label2->TabIndex = 2;
			this->label2->Text = L"X:";
			// 
			// yAxisTextBox
			// 
			this->yAxisTextBox->Location = System::Drawing::Point(54, 37);
			this->yAxisTextBox->Name = L"yAxisTextBox";
			this->yAxisTextBox->Size = System::Drawing::Size(197, 20);
			this->yAxisTextBox->TabIndex = 2;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(31, 40);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(17, 13);
			this->label3->TabIndex = 4;
			this->label3->Text = L"Y:";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(11, 119);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(37, 13);
			this->label4->TabIndex = 6;
			this->label4->Text = L"Mode:";
			// 
			// okayButton
			// 
			this->okayButton->Location = System::Drawing::Point(15, 154);
			this->okayButton->Name = L"okayButton";
			this->okayButton->Size = System::Drawing::Size(75, 23);
			this->okayButton->TabIndex = 6;
			this->okayButton->Text = L"OK";
			this->okayButton->UseVisualStyleBackColor = true;
			this->okayButton->Click += gcnew System::EventHandler(this, &InitialLocationForm::okayButton_Click);
			// 
			// pickAgainButton
			// 
			this->pickAgainButton->Location = System::Drawing::Point(96, 154);
			this->pickAgainButton->Name = L"pickAgainButton";
			this->pickAgainButton->Size = System::Drawing::Size(75, 23);
			this->pickAgainButton->TabIndex = 7;
			this->pickAgainButton->Text = L"Pick Again";
			this->pickAgainButton->UseVisualStyleBackColor = true;
			this->pickAgainButton->Click += gcnew System::EventHandler(this, &InitialLocationForm::pickAgainButton_Click);
			// 
			// cancelButton
			// 
			this->cancelButton->Location = System::Drawing::Point(177, 154);
			this->cancelButton->Name = L"cancelButton";
			this->cancelButton->Size = System::Drawing::Size(75, 23);
			this->cancelButton->TabIndex = 8;
			this->cancelButton->Text = L"Cancel";
			this->cancelButton->UseVisualStyleBackColor = true;
			this->cancelButton->Click += gcnew System::EventHandler(this, &InitialLocationForm::cancelButton_Click);
			// 
			// angleUpDown
			// 
			this->angleUpDown->Location = System::Drawing::Point(54, 90);
			this->angleUpDown->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, 0 });
			this->angleUpDown->Name = L"angleUpDown";
			this->angleUpDown->Size = System::Drawing::Size(197, 20);
			this->angleUpDown->TabIndex = 4;
			// 
			// modeUpDown
			// 
			this->modeUpDown->Location = System::Drawing::Point(54, 119);
			this->modeUpDown->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 4, 0, 0, 0 });
			this->modeUpDown->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			this->modeUpDown->Name = L"modeUpDown";
			this->modeUpDown->Size = System::Drawing::Size(197, 20);
			this->modeUpDown->TabIndex = 5;
			this->modeUpDown->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
			// 
			// zAxisTextBox
			// 
			this->zAxisTextBox->Location = System::Drawing::Point(54, 62);
			this->zAxisTextBox->Name = L"zAxisTextBox";
			this->zAxisTextBox->Size = System::Drawing::Size(197, 20);
			this->zAxisTextBox->TabIndex = 3;
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(31, 65);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(17, 13);
			this->label5->TabIndex = 13;
			this->label5->Text = L"Z:";
			// 
			// InitialLocationForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(264, 195);
			this->Controls->Add(this->zAxisTextBox);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->modeUpDown);
			this->Controls->Add(this->angleUpDown);
			this->Controls->Add(this->cancelButton);
			this->Controls->Add(this->pickAgainButton);
			this->Controls->Add(this->okayButton);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->yAxisTextBox);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->xAxisTextBox);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->MaximizeBox = false;
			this->MinimizeBox = false;
			this->Name = L"InitialLocationForm";
			this->Text = L"Initial Location";
			this->Load += gcnew System::EventHandler(this, &InitialLocationForm::InitialLocationForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->angleUpDown))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->modeUpDown))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void okayButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void pickAgainButton_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void cancelButton_Click(System::Object^  sender, System::EventArgs^  e);

	public: System::Void updateInputs(double x, double y, float angle, int mode);
	public: System::Void updateInputs(InitialLocation location);
	public: InitialLocation getLocation();
	public: System::Void disableRetryButton();
	
private: System::Void InitialLocationForm_Load(System::Object^  sender, System::EventArgs^  e) {
}
};
}
