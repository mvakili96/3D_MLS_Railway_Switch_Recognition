#include "ExtractionParameterForm.h"

System::Void PCLProject::ExtractionParameterForm::okayButton_Click(System::Object ^ sender, System::EventArgs ^ e) 
{
	this->DialogResult = System::Windows::Forms::DialogResult::Yes;
}

System::Void PCLProject::ExtractionParameterForm::cancelButton_Click(System::Object ^ sender, System::EventArgs ^ e)
{
	this->DialogResult = System::Windows::Forms::DialogResult::No;
}

ExtractionParameters PCLProject::ExtractionParameterForm::getValues()
{
	ExtractionParameters parameters;

	String^ trackWindowLengthString = this->trackWindowLenghtTextBox->Text;
	String^ trackWindowWidthString = this->trackWindowWidthTextBox->Text;
	String^ trackGuageString = this->TrackGuageTextBox->Text;
	String^ minTrackPointsString = this->minTrackPointsTextBox->Text;
	String^ minPointsString = this->minPointsTextBox->Text;
	String^ maxDistanceString = this->maxDistanceTextBox->Text;
	String^ distanceGroupingString = this->distanceGroupingTextBox->Text;
	String^ weightParameterString = this->weightParameterTextBox->Text;
	String^ minIntensityRangeString = this->minIntensityRangeTextBox->Text;
	String^ maxIntensityRangeString = this->maxIntensityRangeTextBox->Text;

	parameters.trackWindowLength = (double)Convert::ToDouble(trackWindowLengthString);
	parameters.trackWindowWidth = (double)Convert::ToDouble(trackWindowWidthString);
	parameters.trackGuage = (double)Convert::ToDouble(trackGuageString);
	parameters.minTrackPoints = (double)Convert::ToDouble(minTrackPointsString);
	parameters.minPoints = (double)Convert::ToDouble(minPointsString);
	parameters.maxDistanceForRANSAC = (double)Convert::ToDouble(maxDistanceString);
	parameters.distanceGrouping = (double)Convert::ToDouble(distanceGroupingString);
	parameters.weightParameterForMDL = (double)Convert::ToDouble(weightParameterString);
	parameters.minIntensityRangeOfTrack = (double)Convert::ToDouble(minIntensityRangeString);
	parameters.maxIntensityRangeOfTrack = (double)Convert::ToDouble(maxIntensityRangeString);

	return parameters;
}

void PCLProject::ExtractionParameterForm::updateValues(ExtractionParameters parameters)
{
	this->trackWindowLenghtTextBox->Text = "" + parameters.trackWindowLength;
	this->trackWindowWidthTextBox->Text = "" + parameters.trackWindowWidth;
	this->TrackGuageTextBox->Text = "" + parameters.trackGuage;
	this->minTrackPointsTextBox->Text = "" + parameters.minTrackPoints;
	this->minPointsTextBox->Text = "" + parameters.minPoints;
	this->maxDistanceTextBox->Text = "" + parameters.maxDistanceForRANSAC;
	this->distanceGroupingTextBox->Text = "" + parameters.distanceGrouping;
	this->weightParameterTextBox->Text = "" + parameters.weightParameterForMDL;
	this->minIntensityRangeTextBox->Text = "" + parameters.minIntensityRangeOfTrack;
	this->maxIntensityRangeTextBox->Text = "" + parameters.maxIntensityRangeOfTrack;
}

