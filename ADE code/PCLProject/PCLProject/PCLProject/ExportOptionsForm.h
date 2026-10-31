#pragma once

namespace PCLProject {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for ExportOptionsForm
	/// </summary>
	public ref class ExportOptionsForm : public System::Windows::Forms::Form
	{
	public:
		ExportOptionsForm(void)
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
		~ExportOptionsForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::RadioButton^  gRadioButton;
	private: System::Windows::Forms::RadioButton^  lRadioButton;
	protected:


	private: System::Windows::Forms::Button^  saveButton;

	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::TextBox^  textBox3;
	private: System::Windows::Forms::Label^  label1;

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
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->gRadioButton = (gcnew System::Windows::Forms::RadioButton());
			this->lRadioButton = (gcnew System::Windows::Forms::RadioButton());
			this->saveButton = (gcnew System::Windows::Forms::Button());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->textBox3 = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->gRadioButton);
			this->groupBox1->Controls->Add(this->lRadioButton);
			this->groupBox1->Location = System::Drawing::Point(12, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(405, 78);
			this->groupBox1->TabIndex = 0;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Export as";
			// 
			// gRadioButton
			// 
			this->gRadioButton->AutoSize = true;
			this->gRadioButton->Location = System::Drawing::Point(6, 48);
			this->gRadioButton->Name = L"gRadioButton";
			this->gRadioButton->Size = System::Drawing::Size(193, 21);
			this->gRadioButton->TabIndex = 1;
			this->gRadioButton->Text = L"Global Coordinate System";
			this->gRadioButton->UseVisualStyleBackColor = true;
			this->gRadioButton->CheckedChanged += gcnew System::EventHandler(this, &ExportOptionsForm::gRadioButton_CheckedChanged);
			// 
			// lRadioButton
			// 
			this->lRadioButton->AutoSize = true;
			this->lRadioButton->Checked = true;
			this->lRadioButton->Location = System::Drawing::Point(6, 21);
			this->lRadioButton->Name = L"lRadioButton";
			this->lRadioButton->Size = System::Drawing::Size(186, 21);
			this->lRadioButton->TabIndex = 0;
			this->lRadioButton->TabStop = true;
			this->lRadioButton->Text = L"Local Coordinate System";
			this->lRadioButton->UseVisualStyleBackColor = true;
			this->lRadioButton->CheckedChanged += gcnew System::EventHandler(this, &ExportOptionsForm::lRadioButton_CheckedChanged);
			// 
			// saveButton
			// 
			this->saveButton->Location = System::Drawing::Point(331, 189);
			this->saveButton->Name = L"saveButton";
			this->saveButton->Size = System::Drawing::Size(86, 32);
			this->saveButton->TabIndex = 1;
			this->saveButton->Text = L"Save";
			this->saveButton->UseVisualStyleBackColor = true;
			this->saveButton->Click += gcnew System::EventHandler(this, &ExportOptionsForm::saveButton_Click);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(18, 149);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(100, 22);
			this->textBox1->TabIndex = 2;
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(124, 149);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(100, 22);
			this->textBox2->TabIndex = 3;
			// 
			// textBox3
			// 
			this->textBox3->Location = System::Drawing::Point(230, 149);
			this->textBox3->Name = L"textBox3";
			this->textBox3->Size = System::Drawing::Size(100, 22);
			this->textBox3->TabIndex = 4;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(18, 126);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(120, 17);
			this->label1->TabIndex = 5;
			this->label1->Text = L"Base Coordinates";
			// 
			// ExportOptionsForm
			// 
			this->AcceptButton = this->saveButton;
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(429, 232);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->textBox3);
			this->Controls->Add(this->textBox2);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->saveButton);
			this->Controls->Add(this->groupBox1);
			this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::FixedToolWindow;
			this->Name = L"ExportOptionsForm";
			this->Text = L"Export Options";
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion

	private: System::Void saveButton_Click(System::Object^  sender, System::EventArgs^  e) {
		this->DialogResult = System::Windows::Forms::DialogResult::Yes;
	}

	private: System::Void gRadioButton_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}

	private: System::Void lRadioButton_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	}
};
}
