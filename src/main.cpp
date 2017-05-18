#include <iostream>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>
#include "opencv2/video.hpp"
#include "opencv2/video/background_segm.hpp"
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <chrono>

using namespace sl;


cv::Mat slMat2cvMat(sl::Mat& input);

// Create Mat objects for background subtraction
cv::Mat fgmaskMOG2;
float distSum;
int heightNumber = 1;
float a, smallestDistance_index;
float smallestDistance = 100;
bool isNextProbe = false;
bool hasPicture = false;
bool isWithinBox = false;
bool foundContour = false;
float distMean = 0;
//float smallestDistance = 0;
float receivedDistance = 0;
float dist;
float receivedCb, receivedCr, receivedY, receivedDepth;
double finalDistance;
int k = 1;
int l = 0;
int gal1, gal2, gal3, gal4, gal5, gal6, gal7, gal8, gal9, gal10, gal11, gal12, probeGalDistanceIndex;
int probeGalShortestDistance = 2000;
int r_gal1, r_gal2, r_gal3, r_gal4, r_gal5, r_gal6, r_gal7, r_gal8, r_gal9, r_gal10, r_gal11, r_gal12, r_hour, r_min, r_sec, probeDistance;
float receivedArray[9];
float galArray[12] = { gal1, gal2, gal3, gal4, gal5, gal6, gal7, gal8, gal9, gal10, gal11, gal12 };
float r_galArray[12] = { r_gal1, r_gal2, r_gal3, r_gal4, r_gal5, r_gal6, r_gal7, r_gal8, r_gal9, r_gal10, r_gal11, r_gal12 };
float y_mean, cr_mean, cb_mean = 0;


cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2;


int main(int argc, char **argv) {

	// Create a ZED camera object
	Camera zed;

	// Set configuration parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD2K;
	init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = sl::UNIT_METER;
	init_params.camera_fps = 15;

	// Open the camera
	ERROR_CODE err1 = zed.open(init_params);
	ERROR_CODE err2;
	if (err1 != SUCCESS) {
		init_params.svo_input_filename = ("C:/GitHub/ZEDProject/alex2.svo"), false;
		init_params.svo_real_time_mode = false;
		 err2 = zed.open(init_params);
	}
	zed.setCameraSettings(CAMERA_SETTINGS_SATURATION, 4, false);
	zed.setCameraSettings(CAMERA_SETTINGS_WHITEBALANCE, 2800, false);
	zed.setCameraSettings(CAMERA_SETTINGS_HUE, 0, false);
	zed.setCameraSettings(CAMERA_SETTINGS_CONTRAST, 4, false);
	zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 83, false);
	zed.setCameraSettings(CAMERA_SETTINGS_GAIN, 29, false);
	zed.setCameraSettings(CAMERA_SETTINGS_BRIGHTNESS, 6, false);

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


	pMOG2 = cv::createBackgroundSubtractorMOG2(2000, 20, true); //MOG2 approach

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

			image2.convertTo(image2, -1, 1.4, -30);

			//Conversion to YCC
			cv::cvtColor(image2, image_ocv, CV_BGR2YCrCb);

			//Displaying YCC 
			cv::resize(image_ocv, ycc_display, displaySize);
			imshow("YCC", ycc_display);
			cv::moveWindow("YCC", 0, 500);

			//Apply background subtraction
			pMOG2->apply(image2, fgmaskMOG2);


			cv::threshold(fgmaskMOG2, fgmaskMOG2, 250, 255, CV_THRESH_BINARY);

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

			/*for (int i = 0; i < contours.size(); i++) {
				double a = cv::contourArea(contours[i], false);
				if (a > largest_area) {
					largest_area = a;
					largest_contour_index = i;
				}

			}*/

			for (int j = 0; j < contours.size(); j++) {
				//cv::drawContours(image2, contours, j, CV_RGB(255, 0, 0), 6, 8, hierarchy, 0, cv::Point());
				//cv::drawContours(image2, hull, (int)j, CV_RGB(255, 100, 0), 2, 8, hierarchy, 0, cv::Point());
				bounding_rect = cv::boundingRect(contours[j]);
				cv::rectangle(image2, bounding_rect, cv::Scalar(100, 255, 0), 8);
			}

			int histSize = 256;
			float range[] = { 0, 255 };
			const float* histRange = { range };
			bool uniform = true;
			bool accumulate = false;
			cv::Mat y_hist, cr_hist, cb_hist;

			if (bounding_rect.width > 200 && bounding_rect.height > 200 && bounding_rect.width < 600 && bounding_rect.height < 600) {
				cv::Mat cropImg;


				//cvWaitKey(0);
				if (bounding_rect.x > 700 && bounding_rect.x < 1100 && bounding_rect.y > 50) {

					foundContour = true;
					isWithinBox = true;
					cropImg = image_ocv(bounding_rect);
					cv::imshow("Cropped", cropImg);
					cv::moveWindow("Cropped", 800, 0);

					//Bounding Box Center
					cv::Point center = cv::Point(bounding_rect.x + (bounding_rect.width / 2), bounding_rect.y + (bounding_rect.height / 2));

					cv::circle(image2, center, 20, cv::Scalar(0, 0, 255), -1, 8, 0);

					int x_int = (bounding_rect.x + (bounding_rect.width / 2));
					int y_int = (bounding_rect.y + (bounding_rect.height / 2));

					depth.getValue(x_int, y_int, &dist);

					heightNumber++;

					if (dist > 0 && !std::isnan(dist)) {
						std::cout << "Depth: " << dist << std::endl;
						cv::waitKey(20);
					}

					std::vector<cv::Mat> ycc_planes;
					split(cropImg, ycc_planes);
					cv::Mat y = ycc_planes[0];
					cv::Mat cr = ycc_planes[1];
					cv::Mat cb = ycc_planes[2];


					// Compute the histograms
					cv::calcHist(&y, 1, 0, cv::Mat(), y_hist, 1, &histSize, &histRange, uniform, accumulate);
					cv::calcHist(&cr, 1, 0, cv::Mat(), cr_hist, 1, &histSize, &histRange, uniform, accumulate);
					cv::calcHist(&cb, 1, 0, cv::Mat(), cb_hist, 1, &histSize, &histRange, uniform, accumulate);

					// Draw the histograms for B, G and R
					int hist_w = 512; int hist_h = 400;
					int bin_w = cvRound((double)hist_w / histSize);


					cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

					// Normalize the result to [ 0, histImage.rows ]
					cv::normalize(y_hist, y_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
					cv::normalize(cr_hist, cr_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
					cv::normalize(cb_hist, cb_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

					

					for (int i = 1; i < histSize; i++)
					{
						line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(y_hist.at<float>(i - 1))),
							cv::Point(bin_w*(i), hist_h - cvRound(y_hist.at<float>(i))),
							cv::Scalar(255, 0, 0), 2, 8, 0);

						y_mean = y_mean + cvRound(y_hist.at<float>(i - 1));

						line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(cr_hist.at<float>(i - 1))),
							cv::Point(bin_w*(i), hist_h - cvRound(cr_hist.at<float>(i))),
							cv::Scalar(0, 255, 0), 2, 8, 0);

						cr_mean = cr_mean + cvRound(cr_hist.at<float>(i - 1));

						line(histImage, cv::Point(bin_w*(i - 1), hist_h - cvRound(cb_hist.at<float>(i - 1))),
							cv::Point(bin_w*(i), hist_h - cvRound(cb_hist.at<float>(i))),
							cv::Scalar(0, 0, 255), 2, 8, 0);

						cb_mean = cb_mean + cvRound(cb_hist.at<float>(i - 1));
					}
					y_mean = y_mean / histSize;
					cr_mean = cr_mean / histSize;
					cb_mean = cb_mean / histSize;

					cv::Vec4d feature_Vec{ (double)y_mean,(double)cr_mean,(double)cb_mean, (double)dist };
					cv::Vec4d col_Vec_receive;
					cv::Vec4d receivedVector;
						
					time_t now = time(0);

					tm *ltm = localtime(&now);
					int hour = ltm->tm_hour;
					int min = ltm->tm_min;
					int sec = ltm->tm_sec;

					for (int i = 1; i < 13; i++) {
						std::string galleryString = std::string("Gallery") + std::to_string(i) + std::string(".txt");

						cv::FileStorage fs_receive(galleryString, cv::FileStorage::READ);
						fs_receive["Y"] >> receivedY;
						fs_receive["Cr"] >> receivedCr;
						fs_receive["Cb"] >> receivedCb;
						fs_receive["Depth"] >> receivedDepth;

						receivedVector = { (double)receivedY, (double)receivedCr, (double)receivedCb, (double)receivedDepth };


						if (!std::isnan(dist) && isWithinBox == true) {
							finalDistance = cv::norm(receivedVector, feature_Vec);


							if (finalDistance < smallestDistance) {
								smallestDistance = finalDistance;
								smallestDistance_index = (float)i;
							}

							if (i == 1) {
								gal1 = finalDistance;
							}
							else if (i == 2) {
								gal2 = finalDistance;
							}
							else if (i == 3) {
								gal3 = finalDistance;
							}
							else if (i == 4) {
								gal4 = finalDistance;
							}
							else if (i == 5) {
								gal5 = finalDistance;
							}
							else if (i == 6) {
								gal6 = finalDistance;
							}
							else if (i == 7) {
								gal7 = finalDistance;
							}
							else if (i == 8) {
								gal8 = finalDistance;
							}
							else if (i == 9) {
								gal9 = finalDistance;
							}
							else if (i == 10) {
                                gal10 = finalDistance;
                            }
                            else if (i == 11) {
                                gal11 = finalDistance;
                            }
                            else if (i == 12) {
                                gal12 = finalDistance;
                            }

						}


						//	std::cout << "Received Mean: " << receivedRed << std::endl;

							//col_Vec_receive = (int) fs_receive["Means"];

						fs_receive.release();
					}
										/*
					for (int i = 1; i < 4; i++) {
						std::string probeString = std::string("ProbeGal") + std::to_string(i) + std::string(".txt");
						cv::FileStorage fs_receive(probeString, cv::FileStorage::READ);


						fs_receive["Hour"] >> r_hour;
						fs_receive["Minute"] >> r_min;
						fs_receive["Seconds"] >> r_sec;
						fs_receive["Gallery1"] >> r_gal1;
						fs_receive["Gallery2"] >> r_gal2;
						fs_receive["Gallery3"] >> r_gal3;
						fs_receive["Gallery4"] >> r_gal4;
						fs_receive["Gallery5"] >> r_gal5;
						fs_receive["Gallery6"] >> r_gal6;
						fs_receive["Gallery7"] >> r_gal7;
						fs_receive["Gallery8"] >> r_gal8;
						fs_receive["Gallery9"] >> r_gal9;
						fs_receive["Gallery10"] >> r_gal10;
						fs_receive["Gallery11"] >> r_gal11;
						fs_receive["Gallery12"] >> r_gal12;

						cv::Mat r_galMat(9, 1, CV_32F, r_galArray);
						cv::Mat galMat(9, 1, CV_32F, galArray);

						probeDistance = cv::norm(galMat, r_galMat);

						if (probeDistance < probeGalShortestDistance) {
							probeGalShortestDistance = probeDistance;
							probeGalDistanceIndex = (float)i;
						}



						fs_receive.release();
					}*/
					
					
					std::cout << "Probe distance: " << probeDistance << std::endl;

					if (foundContour == true && hasPicture == false && !std::isnan(dist)&& err1 != SUCCESS) {

						std::string probeString = std::string("Probe") + std::to_string(k) + std::string(".txt");

						cv::FileStorage fs(probeString, cv::FileStorage::WRITE);
						fs << "Hour" << hour;
						fs << "Minute" << min;
						fs << "Seconds" << sec;
						//fs << "Means " << feature_Vec;
						fs << "Y" << (int)y_mean;
						fs << "Cr" << (int)cr_mean;
						fs << "Cb" << (int)cb_mean;
						fs << "Depth" << dist;
						fs << "Closest to" << smallestDistance_index;
						fs << "Distance to gallery" << (int)smallestDistance;
						//	fs << "Shortest Distance " << smallestDistance;
						fs << "Gallery1" << gal1;
						fs << "Gallery2" << gal2;
						fs << "Gallery3" << gal3;
						fs << "Gallery4" << gal4;
						fs << "Gallery5" << gal5;
						fs << "Gallery6" << gal6;
						fs << "Gallery7" << gal7;
						fs << "Gallery8" << gal8;
						fs << "Gallery9" << gal9;
						fs << "Gallery10" << gal10;
						fs << "Gallery11" << gal11;
						fs << "Gallery12" << gal12;
						//fs << "PROBE DISTANCE: " << probeDistance;

						fs.release();
						imshow("calcHist demo", histImage);
						cropImg = image2(bounding_rect);
						cv::imwrite("C:/GitHub/ZEDProject/build/cropImgA" + std::to_string(k) + ".jpg", cropImg);

						k++;
						foundContour = false;
						hasPicture = true;
					
					
					}


					if (dist > 0.8) {
						std::cout << "Euclidean Distance " << std::to_string(smallestDistance_index) << ": " << smallestDistance << std::endl;
					
					}
					

				}
				else {
					y_mean, cr_mean, cb_mean, dist = 0;
					smallestDistance = 100;
					isWithinBox = false;
					isNextProbe = false;
					foundContour = false;
					hasPicture = false;
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
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}