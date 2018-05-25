#include "stdafx.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 

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

	cout << "Grabbed image " << ", width = " << rowsize << ", height = " << colsize << endl;
	cout << "Grabbed image " << ", XPadding = " << XPadding << ", YPadding = " << YPadding << endl;

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
int ContinuousAcquisition(INodeMap & nodeMap)
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
	ContinuousAcquisition(nodeMap);

	// Configure exposure
	double exposureTimeToSet = 2000.0;
	ConfigureExposure(nodeMap, exposureTimeToSet);

	pCam->BeginAcquisition();

	cv::Mat img= AcquireSingleImage(pCam);

	cv::Point2f currentMassCenter = GetSpotCenter(pCam);
	cout << currentMassCenter << endl;

	// deinitialize camera
	pCam->DeInit();
	pCam = NULL;

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();


	return 0;
}

