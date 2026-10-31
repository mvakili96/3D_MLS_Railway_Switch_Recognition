#include "InitialLocationForm.h"

using namespace System;

System::Void PCLProject::InitialLocationForm::okayButton_Click(System::Object ^ sender, System::EventArgs ^ e) {
	this->DialogResult = System::Windows::Forms::DialogResult::Yes;
}

System::Void PCLProject::InitialLocationForm::pickAgainButton_Click(System::Object ^ sender, System::EventArgs ^ e) {
	this->DialogResult = System::Windows::Forms::DialogResult::Retry;
}

System::Void PCLProject::InitialLocationForm::cancelButton_Click(System::Object ^ sender, System::EventArgs ^ e) {
	this->DialogResult = System::Windows::Forms::DialogResult::No;
}

System::Void PCLProject::InitialLocationForm::updateInputs(double x, double y, float angle, int mode)
{
	this->xAxisTextBox->Text = "" + x;
	this->yAxisTextBox->Text = "" + y;
	this->angleUpDown->Value = (int)angle;
	this->modeUpDown->Value = mode;
}

System::Void PCLProject::InitialLocationForm::updateInputs(InitialLocation location)
{

	int mode = location.mode;
	if (mode <= 0 || mode > 4) {
		mode = 1;
	}

	int prevangle = (int)location.angle % 360;
	int angle = (360 + prevangle) % 360;

	this->xAxisTextBox->Text = "" + location.x;
	this->yAxisTextBox->Text = "" + location.y;
	this->zAxisTextBox->Text = "" + location.z;
	this->angleUpDown->Value = angle;
	this->modeUpDown->Value = mode;
}

InitialLocation PCLProject::InitialLocationForm::getLocation()
{
	InitialLocation location;

	String^ xAxisString = this->xAxisTextBox->Text;
	String^ yAxisString = this->yAxisTextBox->Text;
	String^ zAxisString = this->zAxisTextBox->Text;

	location.x = (double)Convert::ToDouble(xAxisString);
	location.y = (double)Convert::ToDouble(yAxisString);
	location.z = (double)Convert::ToDouble(zAxisString);

	location.angle = (double)this->angleUpDown->Value;
	location.mode = (int)this->modeUpDown->Value;

	return location;
}

System::Void PCLProject::InitialLocationForm::disableRetryButton()
{
	this->pickAgainButton->Enabled = false;
}
