//#include <iostream>
#include <sl/Camera.hpp>
//#include <opencv2/opencv.hpp>
//#include <sl/Core.hpp>
//#include <sl/defines.hpp>
//#include "opencv2/video.hpp"
//#include "opencv2/video/background_segm.hpp"
//#include <opencv2/imgproc.hpp>
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include <stdio.h>
//#include <math.h>
//#include <ctime>
//#include <chrono>

using namespace sl;
/*
typedef struct mouseOCVStruct {
	Mat depth;
	cv::Size _resize;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param);
*/

cv::Mat slMat2cvMat(sl::Mat& input);

// Create Mat objects for background subtraction
cv::Mat fgmaskMOG2;
float distSum;
int heightNumber = 0;
bool hasPicture = false;
float distMean = 0;
float smallestDistance = 0;
float receivedDistance = 0;

cv::Ptr<cv::BackgroundSubtractor> pMOG2;

int main(int argc, char **argv) {

	// Create a ZED camera object
	Camera zed;

	time_t now = time(0);

	tm *ltm = localtime(&now);

	// Set configuration parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = sl::UNIT_METER;
	/*
	init_params.camera_fps = 0;
	init_params.coordinate_system = COORDINATE_SYSTEM_IMAGE;
	init_params.sdk_verbose = false;
	init_params.sdk_gpu_id = -1;
	init_params.depth_minimum_distance = -1;
	init_params.camera_disable_self_calib = false;
	init_params.camera_image_flip = false;
	*/

	// Open the camera
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS) {
		init_params.svo_input_filename = ("C:/GitHub/ZEDProject/newestsvo.svo"), false;
		init_params.svo_real_time_mode = true;
		ERROR_CODE err = zed.open(init_params);
	}

	zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 80, false);
	zed.setCameraSettings(CAMERA_SETTINGS_GAIN, 80, false);

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

	pMOG2 = cv::createBackgroundSubtractorMOG2(); //MOG2 approach

	/*
	// Mouse callback initialization 
	mouseStruct.depth.alloc(image_size, MAT_TYPE_32F_C1);
	mouseStruct._resize = displaySize;
	*/
	// Give a name to OpenCV Windows
	cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

	// Setup depth matrix
	Mat depth;
	depth.alloc(image_size, MAT_TYPE_32F_C1);

	// Loop until 'q' is pressed
	char key = ' ';
	while (key != 'q') {

		// Grab and display image and depth 
		if (zed.grab(runtime_parameters) == SUCCESS) {

			zed.retrieveImage(image_zed, VIEW_LEFT); // Retrieve the left image
			zed.retrieveImage(depth_image_zed, VIEW_DEPTH); //Retrieve the depth view (image)
			zed.retrieveMeasure(depth, MEASURE_DEPTH); // Retrieve the depth measure (32bits)

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

			//imshow("Segmentation", fgmaskMOG2);

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
					
				}
				
			}

			for (int j = 0; j < contours.size(); j++) {
				//cv::drawContours(image2, contours, largest_contour_index, CV_RGB(255, 0, 0), 6, 8, hierarchy, 0, cv::Point());
				//cv::drawContours(image2, hull, (int)j, CV_RGB(255, 100, 0), 2, 8, hierarchy, 0, cv::Point());
				bounding_rect = cv::boundingRect(contours[largest_contour_index]);
				cv::rectangle(image2, bounding_rect, cv::Scalar(100, 255, 0), 8);
			}
			
			int histSize = 256;
			float range[] = { 0, 255};
			const float* histRange = { range };
			bool uniform = true; 
			bool accumulate = false;
			cv::Mat b_hist, g_hist, r_hist;

			if (bounding_rect.width > 100 && bounding_rect.height > 400) {
				cv::Mat cropImg = image_ocv(bounding_rect);
				cv::imshow("Cropped", cropImg);
				cv::moveWindow("Cropped", 1600, 0);


				if (bounding_rect.x > 400 && bounding_rect.x < 600) {

					//Saves picture
					if (hasPicture == false) {
						cv::imwrite("C:/GitHub/ZEDProject/build/cropImg.jpg", cropImg);
						hasPicture = true;
					}

					//Bounding Box Center
					cv::Point center = cv::Point(bounding_rect.x + (bounding_rect.width / 2), bounding_rect.y + (bounding_rect.height / 2));

					cv::circle(image2, center, 20, cv::Scalar(0, 0, 255), -1, 8, 0);

					int x_int = (bounding_rect.x + (bounding_rect.width / 2));
					int y_int = (bounding_rect.y + (bounding_rect.height / 2));

					float dist;
					depth.getValue(x_int, y_int, &dist);

					distSum = distSum + dist;

					heightNumber++;
				}


				distMean = distSum / heightNumber;

				if (distMean > 0) {
					std::cout << "Depth: " << distMean << std::endl;
				}

				std::vector<cv::Mat> bgr_planes;
				split(cropImg, bgr_planes);
				cv::Mat blue = bgr_planes[0];
				cv::Mat green = bgr_planes[1];
				cv::Mat red = bgr_planes[2];

				/// Compute the histograms:
				cv::calcHist(&blue, 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
				cv::calcHist(&green, 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
				cv::calcHist(&red, 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

				// Draw the histograms for B, G and R
				int hist_w = 512; int hist_h = 400;
				int bin_w = cvRound((double)hist_w / histSize);


				cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

				/// Normalize the result to [ 0, histImage.rows ]
				cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
				cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
				cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

				int blue_mean, green_mean, red_mean, colorNumber = 0;

				for (int i = 1; i < histSize; i++)
				{
					line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
						cv::Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
						cv::Scalar(255, 0, 0), 2, 8, 0);

					blue_mean = blue_mean + cvRound(b_hist.at<float>(i - 1));

					line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
						cv::Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
						cv::Scalar(0, 255, 0), 2, 8, 0);

					green_mean = green_mean + cvRound(g_hist.at<float>(i - 1));

					line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
						cv::Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
						cv::Scalar(0, 0, 255), 2, 8, 0);

					red_mean = red_mean + cvRound(r_hist.at<float>(i - 1));
					colorNumber++;
				}
				blue_mean = blue_mean / colorNumber;
				green_mean = green_mean / colorNumber;
				red_mean = red_mean / colorNumber;

				cv::Vec4d col_Vec1{ 21.0,14.0,22.0, 2.0 }; //high_exposure_w_scarf
				cv::Vec4d col_Vec2{ 48,5.0,9.0, 1.64 }; //newestsvo2
				cv::Vec4d feature_Vec{ (double)blue_mean,(double)green_mean,(double)red_mean, (double)distMean };
				cv::Vec4d col_Vec_receive;


				if (distMean > 0.2) {
					int high_exposure_w_scarf = cv::norm(col_Vec1, feature_Vec);
					int newestsvo2 = cv::norm(col_Vec2, feature_Vec);
					if (high_exposure_w_scarf < newestsvo2) {
						std::cout << "Closest to: high_exposure_w_scarf: " << high_exposure_w_scarf << std::endl;
						smallestDistance = high_exposure_w_scarf;
					}
					else {
						std::cout << "Closest to: newestsvo2" << newestsvo2 << std::endl;
						smallestDistance = newestsvo2;
					}

					int hour = ltm->tm_hour;
					int min = ltm->tm_min;
					int sec = ltm->tm_sec;

					cv::FileStorage fs("Histogram_Means.txt", cv::FileStorage::WRITE);
					fs << "Hour" << hour;
					fs << "Minute" << min;
					fs << "Seconds" << sec;
					fs << "Means " << feature_Vec;
					fs << "Shortest Distance " << smallestDistance;
					fs.release();
					imshow("calcHist demo", histImage);

					
					cv::FileStorage fs_receive("Histogram_Means.txt", cv::FileStorage::READ);
					fs_receive["Smallest Distance"] >>  receivedDistance;

					std::cout << "Shortest distance: " << receivedDistance << std::endl;

					//col_Vec_receive = (int) fs_receive["Means"];

					fs_receive.release();
					
				}
				else {
					std::cout << "Waiting for data" << distMean << std::endl;
				}
			}

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

/*
//MouseCallback
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
}*/

//Mat Conversion
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