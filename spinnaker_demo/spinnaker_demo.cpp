#include "stdafx.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <Windows.h>

#include <string.h>
#include <tchar.h>
#include <math.h>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

#include "NIDAQmx.h"


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


// convert captured image and return it as a opencv Mat 
cv::Mat AcquireSingleImage(CameraPtr pCam)
{	
	ImagePtr pResultImage = pCam->GetNextImage();

	// Convert image to mono 8
	ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
    
	// release camera buffer
	pResultImage->Release();

	// End acquisition
	pCam->EndAcquisition();
	unsigned int XPadding = convertedImage->GetXPadding();
	unsigned int YPadding = convertedImage->GetYPadding();
	unsigned int rowsize = convertedImage->GetWidth();
	unsigned int colsize = convertedImage->GetHeight();

	cout << "Grabbed image: " << "width = " << rowsize << ", height = " << colsize << endl << endl;
	//cout << "Grabbed image " << ", XPadding = " << XPadding << ", YPadding = " << YPadding << endl;

	// convert to OpenCV Mat
	//cv::Mat image(colsize + YPadding, rowsize + XPadding, CV_8UC1);


	return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride()).clone();
}


// return the coordinate of the largest contour caputured from a input camera
cv::Point2f GetSpotCenter(CameraPtr pCam)
{
	pCam->BeginAcquisition();

	ImagePtr pResultImage = pCam->GetNextImage();

	// Convert image to mono 8
	ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
	// release camera buffer
	pResultImage->Release();

	// End acquisition
	pCam->EndAcquisition();
	unsigned int XPadding = convertedImage->GetXPadding();
	unsigned int YPadding = convertedImage->GetYPadding();
	unsigned int rowsize = convertedImage->GetWidth();
	unsigned int colsize = convertedImage->GetHeight();

	cv::Mat image(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());

	// blur it slightly
	cv::Mat blurred;
	cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0);

	// threshold it
	cv::Mat thresh;
	cv::threshold(image, thresh, 128, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	// find contours in the thresholded image
	cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

	vector< vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;

	cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// find the largest contour and its index
	int largest_area = 0;
	int largest_contour_index = 0;
	cv::Rect bounding_rect;

	for (size_t i = 0; i < contours.size(); i++)  // iterate through each contour.
	{
		double area = contourArea(contours[i]);  //  Find the area of contour

		if (area > largest_area)
		{
			largest_area = area;
			largest_contour_index = i;           //Store the index of largest contour
			bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}


	// Get the moments
	cv::Moments mu;
	mu = moments(contours[largest_contour_index], false);

	// Get the mass centers:
	cv::Point2f mc;
	mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

	// generate a RGB image for display color contours
	cv::Mat img_display;
	cv::cvtColor(image, img_display, CV_GRAY2RGB);

	// draw the largest contour
	cv::drawContours(img_display, contours, largest_contour_index, cv::Scalar(0, 0, 255), CV_FILLED, 8, hierarchy);
	// draw a rectangle over largest object
	rectangle(img_display, bounding_rect, cv::Scalar(0, 255, 0), 1, 8, 0);
	// draw the center point
	cv::circle(img_display, mc, 3, cv::Scalar(255, 0, 0), CV_FILLED);
	// caption the center point
	cv::Point2f cap_pos;
	cap_pos = cv::Point2f(mc.x + 10, mc.y - 10);
	cv::putText(img_display, "mass center", cap_pos, cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(255, 0, 0));
	cv::imshow("contour", img_display);

	// display threshold image
	cv::imshow("threshold", thresh);

	cv::waitKey();

	return mc;
}


// configures a costom exposure time and turn off automatic exposure
// exposure time recorded in microseconds
int ConfigureExposure(INodeMap & nodeMap, double exposureTimeToSet)
{
	// Turn off automatic exposure mode
	CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
	CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
	ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
	cout <<endl << "Automatic exposure disabled..." << endl;

	// Set exposure time manually; exposure time recorded in microseconds
	CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");

	// Ensure desired exposure time does not exceed the maximum
	const double exposureTimeMax = ptrExposureTime->GetMax();
	
	//double exposureTimeToSet = 2000000.0;

	if (exposureTimeToSet > exposureTimeMax)
	{
		exposureTimeToSet = exposureTimeMax;
	}

	ptrExposureTime->SetValue(exposureTimeToSet);

	cout << "Exposure time set to " << exposureTimeToSet << " us..." << endl << endl;

	return 0;
}


// returns the camera to its default state by re - enabling automatic
int ResetExposure(INodeMap & nodeMap)
{
	// Turn automatic exposure back on
	CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
	CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
	ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
	cout << "Automatic exposure enabled..." << endl << endl;
	
	return 0;
}


// prints the device information of the camera
int PrintDeviceInfo(INodeMap & nodeMap)
{
	cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

	FeatureList_t features;
	CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
	
	category->GetFeatures(features);

	FeatureList_t::const_iterator it;
	for (it = features.begin(); it != features.end(); ++it)
	{
		CNodePtr pfeatureNode = *it;
		cout << pfeatureNode->GetName() << " : ";
		CValuePtr pValue = (CValuePtr)pfeatureNode;
		cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
		cout << endl;
	}

	return 0;
}

// set camera to Continuous Acquisition Mode
int ContinuousAcquisitionMode(INodeMap & nodeMap)
{
	// Retrieve enumeration node from nodemap
	CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");

	// Retrieve entry node from enumeration node
	CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");

	// Retrieve integer value from entry node
	int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

	// Set integer value from entry node as new value of enumeration node
	ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

	cout << endl << "Acquisition mode set to continuous..." << endl;

	return 0;
}


// adjust ROI of a input camera
int SetROI(INodeMap & nodeMap, int64_t offsetX, int64_t offsetY, int64_t width, int64_t height)
{
	// apply offset X
	CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
	int64_t OffsetXToSet = offsetX;
	ptrOffsetX->SetValue(OffsetXToSet);
	cout << "offset X set to " << ptrOffsetX->GetValue() << "..." << endl;

	// apply offset Y
	CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
	int64_t OffsetYToSet = offsetY;
	ptrOffsetY->SetValue(OffsetYToSet);
	cout << "offset X set to " << ptrOffsetY->GetValue() << "..." << endl;


	// set width
	CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
	int64_t widthToSet = width;
	ptrWidth->SetValue(widthToSet);
	cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;


	// set height
	CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
	int64_t heightToSet = height;
	ptrHeight->SetValue(heightToSet);
	cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;

	return 0;
}


// reset ROI of a input camera
int ReSetROI(INodeMap & nodeMap)
{
	// set width
	CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
	int64_t widthToSet = ptrWidth->GetMax();
	ptrWidth->SetValue(widthToSet);
	cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;

	// set height
	CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
	int64_t heightToSet = ptrHeight->GetMax();
	ptrHeight->SetValue(heightToSet);
	cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;

	return 0;
}


vector<cv::Point2d> calibrate_controller(TaskHandle taskhandle, CameraPtr pCam, cv::Point2f initMassCenter, int calib_start, int calib_stop, int calib_step, double factor)
{
	vector<cv::Point2d> calib_table(0);
	// calibration loop for pizo_serialno
	for (int j = calib_start; j <= calib_stop; j += calib_step) // calibrate with calib_iter 
	{

		// change voltage
		float64 data = j * factor;
		DAQmxWriteAnalogScalarF64(taskhandle, 0, 0, data, NULL);


		Sleep(sleeptime);

		cv::Point2f tempMassCenter = GetSpotCenter(pCam);

		cout << "temp position: " << tempMassCenter << endl;

		// save current drift to drift table
		calib_table.push_back(tempMassCenter - initMassCenter);

	}
	return calib_table;

}


int setvoltage(TaskHandle taskhandle, float64 voltage)
{

	if (voltage >= 10)
	{
		voltage = 10;
		cout << "voltage too high" << endl;
	}

	if (voltage <= 0)
	{
		voltage = 0;
		cout << "voltage too low" << endl;
	}
	DAQmxWriteAnalogScalarF64(taskhandle, 0, 0, voltage, NULL);
	return 0;
}

int main()
{
	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();

	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();

	unsigned int numCameras = camList.GetSize();
	cout << "Number of cameras detected: " << numCameras << endl;
	
	// Create shared pointer
	CameraPtr pCam = NULL;
	
	pCam = camList.GetByIndex(0);
	cout << endl << "Running camera " << 0 << "..." << endl;

	// Retrieve TL device nodemap and print device information
	INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
	PrintDeviceInfo(nodeMapTLDevice);

	// initialize camera
	pCam->Init();
	
	// Retrieve GenICam nodemap
	INodeMap & nodeMap = pCam->GetNodeMap();

	// set camera to Continuous Acquisition Mode
	ContinuousAcquisitionMode(nodeMap);

	// set ROI
	int64_t OffsetX = 100;
	int64_t OffsetY = 0;
	int64_t ImgWidth = 600;
	int64_t ImgHeight = 540;
	
	SetROI(nodeMap, OffsetX, OffsetY, ImgWidth, ImgHeight);
	//ReSetROI(nodeMap);

	// Configure exposure
	double exposureTimeToSet = 2000.0;
	ConfigureExposure(nodeMap, exposureTimeToSet);

	// pCam->BeginAcquisition();
	// cv::Mat img= AcquireSingleImage(pCam);

	//=============================================================================

	// initialize the adc
	TaskHandle	taskHandle_ao0 = 0;
	TaskHandle	taskHandle_ao1 = 0;

	DAQmxCreateTask("", &taskHandle_ao0);
	DAQmxCreateTask("", &taskHandle_ao1);

	DAQmxCreateAOVoltageChan(taskHandle_ao0, "Dev1/ao0", "", 0.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskHandle_ao1, "Dev1/ao1", "", 0.0, 10.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(taskHandle_ao0);
	DAQmxStartTask(taskHandle_ao1);

	//=============================================================================
	
	cv::Point2f initMassCenter;
	cv::Point2f currentMassCenter;
	cv::Point2f correctedMassCenter;

	std::vector<double> position_error_x(0);
	std::vector<double> position_error_y(0);
	std::vector<double> position_error_distance(0);

	// get init position for the spot
	initMassCenter = GetSpotCenter(pCam);
	cout << "initial Mass Center of the largest object: " << initMassCenter << endl;

	//=============================================================================

	// for voltage calibration
	int calib_start = 1;
	int calib_step = 1;
	int calib_stop = 100;

	// initialize the drift val table
	vector<cv::Point2d> drift_table_0(0), drift_table_1(0);
	drift_table_0 = calibrate_controller(taskHandle_ao0, camera, initMassCenter, calib_start, calib_stop, calib_step, 0.1);
	cout << "calibrate next controller: " << endl;
	drift_table_0 = calibrate_controller(taskHandle_ao1, camera, initMassCenter, calib_start, calib_stop, calib_step, 0.1);

	DAQmxStopTask(taskHandle_ao0);
	DAQmxStopTask(taskHandle_ao1);

	// save calibration data to file
	std::vector<double> offset_x(0);
	std::vector<double> offset_y(0);
	ofstream outputFile;
	outputFile.open("outputFile_controller0.txt", std::ios::out);
	for (size_t ii = 0; ii < drift_table_0.size(); ++ii)
	{
		outputFile << drift_table_0[ii].x << "," << drift_table_0[ii].y << std::endl;
		offset_x.push_back(drift_table_0[ii].x);
		offset_y.push_back(drift_table_0[ii].y);
	}
	outputFile.close();

	//=============================================================================

	// find the correspondence between the port and offset 
	cv::Mat x_mean_mat, y_mean_mat, x_std_mat, y_std_mat;
	double x_std, y_std, x_mean, y_mean;
	cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
	cv::meanStdDev(offset_y, y_mean_mat, y_std_mat);
	x_std = x_std_mat.at<double>(0, 0);
	y_std = y_std_mat.at<double>(0, 0);
	//x_mean = x_mean_mat.at<double>(0, 0);
	//y_mean = y_mean_mat.at<double>(0, 0);
	x_mean = offset_x[offset_x.size() - 1] - offset_x[5];
	y_mean = offset_y[offset_y.size() - 1] - offset_y[5];

	short sign_x = 1, sign_y = 1;

	// if the controller[0] controls y axis
	if (x_std < y_std)
	{
		DAQmxCreateAOVoltageChan(taskHandle_ao0, "Dev1/ao1", "", 0.0, 10.0, DAQmx_Val_Volts, "");
		DAQmxCreateAOVoltageChan(taskHandle_ao1, "Dev1/ao0", "", 0.0, 10.0, DAQmx_Val_Volts, "");

		if (y_mean < 0)
		{
			sign_y = -1;
		}
	}
	else
	{
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}

	// save calibration data to file
	outputFile.open("outputFile_controller1.txt", std::ios::out);
	offset_x.clear();
	offset_y.clear();
	for (size_t ii = 0; ii < drift_table_1.size(); ++ii)
	{
		outputFile << drift_table_1[ii].x << "," << drift_table_1[ii].y << std::endl;
		offset_x.push_back(drift_table_1[ii].x);
		offset_y.push_back(drift_table_1[ii].y);
	}
	outputFile.close();
	cv::meanStdDev(offset_x, x_mean_mat, x_std_mat);
	cv::meanStdDev(offset_y, y_mean_mat, y_std_mat);
	//x_mean = x_mean_mat.at<double>(0, 0);
	//y_mean = y_mean_mat.at<double>(0, 0);
	x_mean = offset_x[offset_x.size() - 1] - offset_x[5];
	y_mean = offset_y[offset_y.size() - 1] - offset_y[5];

	if (x_std < y_std)
	{
		if (x_mean < 0)
		{
			sign_x = -1;
		}
	}
	else
	{
		if (y_mean < 0)
		{
			sign_y = -1;
		}
	}

	DAQmxStartTask(taskHandle_ao0);
	DAQmxStartTask(taskHandle_ao1);

	//=============================================================================

	// capture loop
	char key = 0;
	
	float64 error_tolerance = 0.5;
	float64 kp_0 = 5, kp_1 = 5;    // Proportion
	float64 ki_0 = 5.0, ki_1 = 5.0; // Integral    
	float64 kd_0 = 0.0, kd_1 = 0.0; // Derivative
	
	while (key != 'q')
	{
		cv::Point2f currentMassCenter = GetSpotCenter(pCam);
		cout << "Mass Center of the largest object: " << currentMassCenter << endl << endl;


		float64 v0 = 5.0, v1 = 5.0;
		float64 x_error_present = currentMassCenter.x - initMassCenter.x;
		float64 y_error_present = currentMassCenter.y - initMassCenter.y;
		float64 position_error_present = sqrt(pow(x_error_present, 2) + pow(y_error_present, 2));
		float64 delta_v0 = 0, delta_v1 = 0;

		float64 x_error_last = 0, y_error_last = 0;
		float64 x_error_previous = 0, y_error_previous = 0;

		int run_times = 0;
		while (position_error_present > error_tolerance && run_times < 300)
		{


			delta_v0 = kp_0 * (x_error_present - x_error_last)
				     + ki_0 * x_error_present
				     + kd_0 * (x_error_present - 2 * x_error_last + x_error_previous);
			
			delta_v1 = kp_1 * (y_error_present - y_error_last)
				     + ki_1 * y_error_present
				     + kd_1 * (y_error_present - 2 * y_error_last + y_error_previous);
			
			v0 -= sign_x * delta_v0;
			v1 -= sign_y * delta_v1;

			run_times++;

			// set output voltage
			setvoltage(taskHandle_ao0, v0);
			setvoltage(taskHandle_ao1, v1);
			Sleep(sleeptime);

			// read the previous corrected position
			correctedMassCenter = GetSpotCenter(camera);
			//historyMassCenter.push_back(correctedMassCenter);

			// x location of the corrected mass center
			//xloc = correctedMassCenter.x;		
			x_error_previous = x_error_last;
			x_error_last = x_error_present;
			x_error_present = correctedMassCenter.x - initMassCenter.x;
			//yloc = correctedMassCenter.y;
			y_error_previous = y_error_last;
			y_error_last = y_error_present;
			y_error_present = correctedMassCenter.y - initMassCenter.y;
			position_error_present = sqrt(pow(x_error_present, 2) + pow(y_error_present, 2));

			position_error_x.push_back(x_error_present);
			position_error_y.push_back(y_error_present);
			position_error_distance.push_back(position_error_present);
			cout << "iteration counts: " << position_error_distance.size() << endl;
		}

		//for debug 
		fstream outputFile;
		outputFile.open("output_error.txt", std::ios::out);
		for (short ii = 0; ii < position_error_distance.size(); ii++)
		{
			outputFile << position_error_x[ii] << "," << position_error_y[ii] << "," << position_error_distance[ii] << std::endl;
		}
		outputFile.close();

		// clear x,y,distance error log
		position_error_x.clear();
		position_error_y.clear();
		position_error_distance.clear();


	}

	//=============================================================================

	// Reset exposure
	ResetExposure(nodeMap);

	// deinitialize camera
	pCam->DeInit();
	pCam = NULL;

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();

	//=============================================================================

	DAQmxStopTask(taskHandle_ao0);
	DAQmxStopTask(taskHandle_ao1);


	return 0;
}

