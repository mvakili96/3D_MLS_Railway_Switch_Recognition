#include "DataConverstionForm.h"
#include <msclr/marshal.h>
#include "TXTFileReader.h"

using namespace System::IO;
using namespace System::Runtime::InteropServices;
using namespace System::Windows::Forms;

System::Void PCLProject::DataConverstionForm::sourceButton_Click(System::Object^  sender, System::EventArgs^  e) {
	if (this->openFileDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		msclr::interop::marshal_context oMarshalContext;
		cli::array<String ^> ^fileNames = this->openFileDialog->FileNames;
		//const char* pFolderName = oMarshalContext.marshal_as<const char*>(folderName);

		String ^ ext = Path::GetExtension(fileNames[0]);

		/*if (ext->Equals(".las"))
		{
			utmConvertButton->Enabled = true;
			ecefConvertButton->Enabled = false;
		}
		else 
		{
			utmConvertButton->Enabled = false;
			ecefConvertButton->Enabled = true;
		}*/

		utmConvertButton->Enabled = true;
		ecefConvertButton->Enabled = true;

		this->fileProgressLabel->Text = "0 / " + fileNames->Length;
		String ^ result = String::Join(";", fileNames);

		sourceFilesFolderTextBox->Text = result;
	}
}

System::Void PCLProject::DataConverstionForm::destinationButton_Click(System::Object^  sender, System::EventArgs^  e) {
	if (this->folderBrowserDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		msclr::interop::marshal_context oMarshalContext;
		String ^folderName = this->folderBrowserDialog->SelectedPath;
		const char* pFolderName = oMarshalContext.marshal_as<const char*>(folderName);

		destinationFolderTextBox->Text = folderName;
	}
}

System::Void PCLProject::DataConverstionForm::utmConvertButton_Click(System::Object^  sender, System::EventArgs^  e) {

	if (CheckErrors())
	{
		return;
	}

	String ^zoneString = this->zoneTextBox->Text;
	int zone = (int)Convert::ToInt32(zoneString);

	bool hasDestination = false;

	String ^destinationFolderString = this->destinationFolderTextBox->Text;
	hasDestination = Directory::Exists(destinationFolderString);

	String ^sourceFilesString = this->sourceFilesFolderTextBox->Text;
	cli::array<wchar_t> ^splitors = { ';' };
	cli::array<String ^> ^ sourceFileNames = sourceFilesString->Split(splitors);
	
	cli::array<String ^> ^ newFileNames = gcnew cli::array<String ^>(sourceFileNames->Length);

	msclr::interop::marshal_context oMarshalContext;

	size_t totalSteps = 3 * sourceFileNames->Length;
	size_t progressSteps = 0;

	for (int i = 0; i < sourceFileNames->Length; i++)
	{
		String ^fileName = sourceFileNames[i];
		if (File::Exists(fileName))
		{
			String^ ext = Path::GetExtension(fileName);
			
			bool isCSV = ext->Equals(".csv");

			const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
			String^ newFileName = destinationFolderString + Path::DirectorySeparatorChar + Path::GetFileNameWithoutExtension(sourceFileNames[i]) + "_ecef." + (isCSV ? "csv" : "pts");

			const char* pNewFileName = oMarshalContext.marshal_as<const char*>(newFileName);
			//std::cout << "file: " << pFileName << std::endl;
			//std::cout << "new file: " << pNewFileName << std::endl;

			newFileNames[i] = newFileName;

			ProgressCallback^ callback = gcnew ProgressCallback(this, &DataConverstionForm::UpdateProgress);
			GCHandle gch = GCHandle::Alloc(callback);

			IntPtr stubPointer = Marshal::GetFunctionPointerForDelegate(callback);
			UnmanagedProgressCallback functionPointer = static_cast<UnmanagedProgressCallback>(stubPointer.ToPointer());

			pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;

			if (ext->Equals(".las"))
			{
				LASFileReader *reader = new LASFileReader();
				point_cloud = reader->ReadData(pFileName, functionPointer);
			}
			else if (ext->Equals(".pts"))
			{
				TXTFileReader *txtReader = new TXTFileReader();
				point_cloud = txtReader->ReadData(pFileName, functionPointer);
			}
			else 
			{
				TXTFileReader* txtReader = new TXTFileReader();
				point_cloud = txtReader->ReadDataCSV(pFileName, functionPointer);
			}

			String ^filePrefix = (i + 1) +" / " + sourceFileNames->Length;

			this->fileProgressLabel->Text = filePrefix + " - Reading Point Cloud";
			
			progressSteps++;
			UpdateAllFilesProgress(progressSteps, totalSteps);
			

			DataConvertor *converter = new DataConvertor(point_cloud);
			this->fileProgressLabel->Text = filePrefix + " - Converting Points";
			converter->ConvertECEFToUTM(zone, functionPointer);

			progressSteps++;
			UpdateAllFilesProgress(progressSteps, totalSteps);

			this->fileProgressLabel->Text = filePrefix + " - Saving to file";
			converter->SaveConvertedPoints(pNewFileName, functionPointer, (isCSV ? "," : " "));

			progressSteps++;
			UpdateAllFilesProgress(progressSteps, totalSteps);
			
			gch.Free();
		}
	}

	this->fileProgressBar->Value = 0;
	this->allFilesProgressBar->Value = 0;
	this->fileProgressLabel->Text = "";

	MessageBox::Show("Converstion Completed.", "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
}

System::Void PCLProject::DataConverstionForm::ecefConvertButton_Click(System::Object^  sender, System::EventArgs^  e) 
{
	if (CheckErrors())
	{
		return;
	}

	String ^zoneString = this->zoneTextBox->Text;
	int zone = (int)Convert::ToInt32(zoneString);

	bool hasDestination = false;

	String ^destinationFolderString = this->destinationFolderTextBox->Text;
	hasDestination = Directory::Exists(destinationFolderString);

	String ^sourceFilesString = this->sourceFilesFolderTextBox->Text;
	cli::array<wchar_t> ^splitors = { ';' };
	cli::array<String ^> ^ sourceFileNames = sourceFilesString->Split(splitors);

	cli::array<String ^> ^ newFileNames = gcnew cli::array<String ^>(sourceFileNames->Length);

	msclr::interop::marshal_context oMarshalContext;

	size_t totalSteps = 3 * sourceFileNames->Length;
	size_t progressSteps = 0;

	for (int i = 0; i < sourceFileNames->Length; i++)
	{
		String ^fileName = sourceFileNames[i];
		if (File::Exists(fileName))
		{
			String^ ext = Path::GetExtension(fileName);

			bool isCSV = ext->Equals(".csv");

			const char* pFileName = oMarshalContext.marshal_as<const char*>(fileName);
			String ^newFileName = destinationFolderString + Path::DirectorySeparatorChar + Path::GetFileNameWithoutExtension(sourceFileNames[i]) + "_ecef." + (isCSV ? "csv" : "pts");
			const char* pNewFileName = oMarshalContext.marshal_as<const char*>(newFileName);
			//std::cout << "file: " << pFileName << std::endl;
			//std::cout << "new file: " << pNewFileName << std::endl;

			newFileNames[i] = newFileName;

			ProgressCallback^ callback = gcnew ProgressCallback(this, &DataConverstionForm::UpdateProgress);
			GCHandle gch = GCHandle::Alloc(callback);

			IntPtr stubPointer = Marshal::GetFunctionPointerForDelegate(callback);
			UnmanagedProgressCallback functionPointer = static_cast<UnmanagedProgressCallback>(stubPointer.ToPointer());

			
			pcl::PointCloud<PointXYZDouble>::Ptr point_cloud;

			if (ext->Equals(".las"))
			{
				LASFileReader *reader = new LASFileReader();
				point_cloud = reader->ReadData(pFileName, functionPointer);
			}
			else if (ext->Equals(".pts"))
			{
				TXTFileReader* txtReader = new TXTFileReader();
				point_cloud = txtReader->ReadData(pFileName, functionPointer);
			}
			else
			{
				TXTFileReader* txtReader = new TXTFileReader();
				point_cloud = txtReader->ReadDataCSV(pFileName, functionPointer);
			}

			String ^filePrefix = (i + 1) + " / " + sourceFileNames->Length;

			this->fileProgressLabel->Text = filePrefix + " - Reading Point Cloud";

			DataConvertor *converter = new DataConvertor(point_cloud);

			progressSteps++;
			UpdateAllFilesProgress(progressSteps, totalSteps);

			this->fileProgressLabel->Text = filePrefix + " - Converting Points";
			progressSteps++;

			UpdateAllFilesProgress(progressSteps, totalSteps);
			converter->ConvertUTMToECEF(zone, false, functionPointer);

			this->fileProgressLabel->Text = filePrefix + " - Saving to file";
			converter->SaveConvertedPoints(pNewFileName, functionPointer, (isCSV ? "," : " "));

			progressSteps++;
			UpdateAllFilesProgress(progressSteps, totalSteps);

			gch.Free();
		}
	}

	this->fileProgressBar->Value = 0;
	this->allFilesProgressBar->Value = 0;
	this->fileProgressLabel->Text = "";

	MessageBox::Show("Converstion Completed.", "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
}

void PCLProject::DataConverstionForm::UpdateProgress(double readPoints, double totalPoints)
{
	this->fileProgressBar->Minimum = 0;
	this->fileProgressBar->Maximum = totalPoints;
	this->fileProgressBar->Value = readPoints;
}

void PCLProject::DataConverstionForm::UpdateAllFilesProgress(double readPoints, double totalPoints)
{
	this->allFilesProgressBar->Minimum = 0;
	this->allFilesProgressBar->Maximum = totalPoints;
	this->allFilesProgressBar->Value = readPoints;
}

bool PCLProject::DataConverstionForm::CheckErrors() {
	
	bool errors = false;
	String ^message = "";

	if (this->sourceFilesFolderTextBox->Text->Length <= 0)
	{
		errors = true;
		message += "Please select source files";
	}
	
	if (this->destinationFolderTextBox->Text->Length <= 0)
	{
		errors = true;
		message += "\nPlease select destination folder";
	}

	if (this->zoneTextBox->Text->Length <= 0)
	{
		errors = true;
		message += "\nPlease enter a zone value";
	}

	if (errors)
	{
		MessageBox::Show(message, "Errors", MessageBoxButtons::OK, MessageBoxIcon::Error);
	}

	return errors;
}
