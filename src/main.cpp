///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/***************************************************************************************************
** This sample demonstrates how to grab images and depth/disparity map with the ZED SDK          **
** Both images and depth/disparity map are displayed with OpenCV                                 **
** Most of the functions of the ZED SDK are linked with a key press event (using OpenCV)         **
***************************************************************************************************/


#include <iostream>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include "opencv2/video.hpp"
#include "opencv2/video/background_segm.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace sl;

typedef struct mouseOCVStruct {
	Mat depth;
	cv::Size _resize;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);
cv::Mat slMat2cvMat(sl::Mat& input);

// Create Mat objects for background subtraction
cv::Mat currentframe;
cv::Mat fgmaskMOG2;

cv::Ptr<cv::BackgroundSubtractor> pMOG2;

int main(int argc, char **argv) {

	// Create a ZED camera object
	Camera zed;

	// Set configuration parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = sl::UNIT_METER;
	init_params.svo_input_filename = ("C:/GitHub/ZEDProject/newestsvo.svo"), false;
	//init_params.camera_fps = 0;
	/*init_params.svo_real_time_mode = false;
	init_params.coordinate_system = COORDINATE_SYSTEM_IMAGE;
	init_params.sdk_verbose = false;
	init_params.sdk_gpu_id = -1;
	init_params.depth_minimum_distance = -1;
	init_params.camera_disable_self_calib = false;
	init_params.camera_image_flip = false;
	*/
	// Open the camera
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS)
		return 1;

	// Set runtime parameters after opening the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD; // Use STANDARD sensing mode

															 // Create sl and cv Mat to get ZED left image and depth image
															 // Best way of sharing sl::Mat and cv::Mat :
															 // Create a sl::Mat and then construct a cv::Mat using the ptr to sl::Mat data.
	Resolution image_size = zed.getResolution();
	sl::Mat image_zed(image_size, sl::MAT_TYPE_8U_C4); // Create a sl::Mat to handle Left image
	cv::Mat image2 = slMat2cvMat(image_zed); //New mat
	cv::Mat image_ocv = slMat2cvMat(image_zed);
	sl::Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);


	// Create OpenCV images to display (lower resolution to fit the screen)
	cv::Size displaySize(600, 400);
	cv::Mat image_ocv_display(displaySize, CV_8UC4);
	cv::Mat depth_image_ocv_display(displaySize, CV_8UC4);
	cv::Mat yC_ocv_display(displaySize, CV_8UC4);
	cv::Mat contour_display(displaySize, CV_8UC4);
	cv::Mat ycc_display(displaySize, CV_8UC4);

	//create GUI windows
	cv::namedWindow("FG Mask MOG 2");

	pMOG2 = cv::createBackgroundSubtractorMOG2(); //MOG2 approach

												  // Mouse callback initialization
	mouseStruct.depth.alloc(image_size, MAT_TYPE_32F_C1);
	mouseStruct._resize = displaySize;

	// Give a name to OpenCV Windows
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("Depth", onMouseCallback, (void*)&mouseStruct);

	// Jetson only. Execute the calling thread on 2nd core
	Camera::sticktoCPUCore(2);

	// Loop until 'q' is pressed
	char key = ' ';
	while (key != 'q') {

		// Grab and display image and depth 
		if (zed.grab(runtime_parameters) == SUCCESS) {

			//zed.retrieveImage(image_zed2, VIEW_RIGHT_UNRECTIFIED_GRAY);
			zed.retrieveImage(image_zed, VIEW_LEFT); // Retrieve the left image
			zed.retrieveImage(depth_image_zed, VIEW_DEPTH); //Retrieve the depth view (image)
			zed.retrieveMeasure(mouseStruct.depth, MEASURE_DEPTH); // Retrieve the depth measure (32bits)

																   // Displays RGB
			cv::resize(image2, image_ocv_display, displaySize);
			imshow("RGB", image_ocv_display);
			cv::moveWindow("RGB", 800, 0);

			//Displays Depth
			cv::resize(depth_image_ocv, depth_image_ocv_display, displaySize);
			imshow("Depth", depth_image_ocv_display);
			cv::moveWindow("Depth", 0, 0);

			//Conversion to YCC
			cv::cvtColor(image2, image_ocv, CV_BGR2YCrCb);

			//Splitting
			cv::Mat ycc[3];
			cv::split(image_ocv, ycc);

			cv::Mat y = ycc[0] * 0.6;
			cv::Mat cb = ycc[1];
			cv::Mat cr = ycc[2];
			ycc[0] = y;

			//Merging channels
			cv::merge(ycc, 3, image_ocv);

			//Displaying YCC 
			cv::resize(image_ocv, ycc_display, displaySize);
			imshow("YCC", ycc_display);
			cv::moveWindow("YCC", 0, 500);

			//Apply background subtraction
			pMOG2->apply(image_ocv, fgmaskMOG2);

			//Closing
			cv::Mat element = cv::Mat::ones(10, 10, CV_8UC1); //Kernel
			cv::erode(fgmaskMOG2, fgmaskMOG2, element);
			cv::dilate(fgmaskMOG2, fgmaskMOG2, element);

			//opening
			cv::dilate(fgmaskMOG2, fgmaskMOG2, element);
			cv::erode(fgmaskMOG2, fgmaskMOG2, element);

			imshow("Segmentation", fgmaskMOG2);

			//cv::bitwise_not(fgmaskMOG2, fgmaskMOG2);

			//cv::cvtColor(fgmaskMOG2, fgmaskMOG2, CV_BGR2GRAY);

			//cv::Canny(fgmaskMOG2, fgmaskMOG2, 75, 150, 3);
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Vec4i> hierarchy;
			std::vector<std::vector<cv::Point>> hull(contours.size());
			cv::Mat drawing = cv::Mat::zeros(fgmaskMOG2.size(), CV_8UC3);
			int largest_area, largest_contour_index = 0;
			cv::Rect bounding_rect;

			cv::findContours(fgmaskMOG2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

			for (int i = 0; i < contours.size(); i++) {
				double a = cv::contourArea(contours[i], false);
				if (a > largest_area) {
					largest_area = a;
					largest_contour_index = i;
					bounding_rect = cv::boundingRect(contours[i]);
				}
				//cv::rectangle(image2,bounding_rect, cv::Scalar(100, 255, 0),10);

			}
			/*
			for (int j = 0; j < contours.size(); j++) {
			cv::convexHull(contours[j], hull[j], false);
			}*/

			for (int j = 0; j < contours.size(); j++) {
				cv::drawContours(image2, contours, (int)j, CV_RGB(255, 100, 0), -1, 8, hierarchy, 0, cv::Point());
				cv::drawContours(image2, hull, (int)j, CV_RGB(255, 100, 0), 2, 8, hierarchy, 0, cv::Point());
			}



			/*cv::SimpleBlobDetector::Params params;
			//params.minDistBetweenBlobs = 10.0;  // minimum 10 pixels between blobs
			//	params.filterByArea = true;         // filter my blobs by area of blob
			//params.minArea = 20;              // min 20 pixels squared
			//params.maxArea = 500.0;             // max 500 pixels squared
			cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
			std::vector<cv::KeyPoint> myBlobs;
			detector->detect(fgmaskMOG2, myBlobs);
			*/


			/*cv::Mat imWithKeypoints;
			cv::drawKeypoints(fgmaskMOG2, myBlobs, imWithKeypoints, cv::Scalar(0, 0, 255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			imshow("blobs", imWithKeypoints);*/

			//cv::inRange(fgmaskMOG2, cv::Scalar(0, 0, 0), cv::Scalar(0, 255, 255), image_ocv);

			//get the frame number and write it on the current frame
			std::stringstream ss;
			rectangle(image_ocv_display, cv::Point(10, 2), cv::Point(100, 20),
				cv::Scalar(255, 255, 255), -1);
			//ss << zed.get(cv::CAP_PROP_POS_FRAMES);
			std::string frameNumberString = ss.str();
			putText(image_ocv_display, frameNumberString.c_str(), cv::Point(15, 15),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

			//show the current frame and the fg masks
			cv::resize(image2, contour_display, displaySize);
			cv::imshow("Contour Mask", contour_display);
			cv::moveWindow("Contour Mask", 800, 500);



			key = cv::waitKey(10);
		}
	}

	zed.close();
	return 0;
}

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
	if (event == CV_EVENT_LBUTTONDOWN) {
		mouseOCVStruct* data = (mouseOCVStruct*)param;
		int y_int = (y * data->depth.getHeight() / data->_resize.height);
		int x_int = (x * data->depth.getWidth() / data->_resize.width);

		sl::float1 dist;
		data->depth.getValue(x_int, y_int, &dist);

		std::cout << std::endl;
		if (isValidMeasure(dist))
			std::cout << "Depth at (" << x_int << "," << y_int << ") : " << dist << "m";
		else {
			std::string depth_status;
			if (dist == TOO_FAR) depth_status = ("Depth is too far.");
			else if (dist == TOO_CLOSE) depth_status = ("Depth is too close.");
			else depth_status = ("Depth not available");
			std::cout << depth_status;
		}
		std::cout << std::endl;
	}
}


cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}