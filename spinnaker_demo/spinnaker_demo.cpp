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

cv::Mat AcquireSingleImage(CameraPtr pCam)
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

	cout << "Grabbed image " << ", width = " << rowsize << ", height = " << colsize << endl;
	cout << "Grabbed image " << ", XPadding = " << XPadding << ", YPadding = " << YPadding << endl;

	// convert to OpenCV Mat
	//cv::Mat image(colsize + YPadding, rowsize + XPadding, CV_8UC1);


	return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride()).clone();
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
	cout << "Number of cameras detected: " << numCameras << endl << endl;

	CameraPtr pCam = NULL;
	pCam = camList.GetByIndex(0);
	cout << endl << "Running camera " << 0 << "..." << endl;

	pCam->Init();

	cv::Mat img= AcquireSingleImage(pCam);

	pCam->DeInit();

	// display captured image
	cv::namedWindow("image");
	cv::imshow("image", img);
	cv::waitKey();

	pCam = NULL;

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();


	return 0;
}

