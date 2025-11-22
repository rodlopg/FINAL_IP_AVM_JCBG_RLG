#include "pattern.h"
#include <iostream>
#include "patterndetector.h"
#include <opencv2/imgcodecs.hpp>


using namespace cv;
using namespace std;

namespace ARma {
	// Load once (global static)
	static cv::Mat patternImage = cv::imread("alexis_sentado.png", cv::IMREAD_UNCHANGED); // Added
	static Mat useImage = patternImage.clone(); //Added
	Pattern::Pattern(double param1){
		id =-1;
		size = param1;
		orientation = -1;
		confidence = -1;

		rotVec = (Mat_<float>(3,1) << 0, 0, 0);
		transVec = (Mat_<float>(3,1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);


	}

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);		
	}

	void Pattern::showPattern()
	{
		cout << "Pattern ID: " << id << endl;
		cout << "Pattern Size: " << size << endl;
		cout << "Pattern Confedince Value: " << confidence << endl;
		cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
		cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i<3; i++){
		cout << rotMat.at<float>(i,0) << "\t" << rotMat.at<float>(i,1) << "\t" << rotMat.at<float>(i,2) << " |\t"<< transVec.at<float>(i,0) << endl;
		}
	}

	void Pattern::getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{
		Mat intrinsics = cameraMatrix;
		Mat distCoeff = distortions;
		Mat &rot = rotVec;
		Mat &tra = transVec;

		Point2f pat2DPts[4];
		for (int i = 0; i<4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}

		//3D points in pattern coordinate system
		Point3f pat3DPts[4];
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;

		Mat objectPts( 4, 3, CV_32FC1, pat3DPts);
		Mat imagePts(4, 2, CV_32FC1, pat2DPts);
		solvePnP(objectPts, imagePts, intrinsics, distCoeff, rot, tra);
	}

	
	/*|||||||| CHANGED*/

	bool activated1 = false; //Added
	bool activated2 = false; //Added
	bool activated3 = false; //Added
	bool activated4 = false; //Added

	bool button1 = false; //Added
	bool button2 = false; //Added
	bool button3 = false; //Added

	int frames = 0; //Added

	int lifetime1 = 0; //Added
	int lifetime2 = 0; //Added
	int lifetime3 = 0; //Added
	int lifetime4 = 0; //Added

	const int MAX_LIFETIME = 5;   // how many frames a marker stays "active" after being seen //Added

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix)
	{

		Scalar color = Scalar(255, 255, 255);
		//model 3D points: they must be projected to the image plane
		int op = 0;
		
		Mat modelPts1;
		// decay lifetimes every frame
		lifetime1 = std::max(0, lifetime1 - 1); //Added
		lifetime2 = std::max(0, lifetime2 - 1); //Added
		lifetime3 = std::max(0, lifetime3 - 1); //Added
		lifetime4 = std::max(0, lifetime4 - 1); //Added

		// refresh lifetime of detected marker
		if (id == 1) lifetime1 = MAX_LIFETIME; //Added
		if (id == 2) lifetime2 = MAX_LIFETIME; //Added
		if (id == 3) lifetime3 = MAX_LIFETIME; //Added
		if (id == 4) lifetime4 = MAX_LIFETIME; //Added

		// marker is considered active if its lifetime is above 0
		activated1 = (lifetime1 > 0); //Added
		activated2 = (lifetime2 > 0); //Added
		activated3 = (lifetime3 > 0); //Added
		activated4 = (lifetime4 > 0); //Added

		// compute buttons
		button1 = activated1 && activated4; //Added
		button2 = activated3 && activated4; //Added
		button3 = activated2 && activated3; //Added


		


		switch (id) {
			case 1: { color = Scalar(255, 255, 10); break; }
			case 2:	{ color = Scalar(0, 0, 255); break;	}
			case 3:	{ color = Scalar(255, 255, 0); break;	}
			case 4:	{ color = Scalar(255, 10, 255); break;	}
			case 5: { color = Scalar(255, 100, 90); break; }
			default:
				break;

		}

		Mat modelPts;

		switch (id) {
		case 2: {
			//Cool S
			float w = size;          // width of body
			float h = size * 1.6f;   // body height
			float d = size * 0.4f;   // depth (thickness)
			float c = 50;

			modelPts = (Mat_<float>(32, 3) <<
				// line 1 front
				c, c - (h/5), c,
				c, c, c,
				c + (w / 3), c + (h / 5), c,
				c + (w / 3), c + (2*h / 5), c,

				c, c + (3 * h / 5), c,
				c - (w / 3), c + (2 * h / 5), c,
				c - (w / 3), c + (h / 5), c,
				c - (w / 6), c + (h / 10), c,


				// line 2 front
				c, c + (2 * h / 5), c,
				c, c + (h / 5), c,
				c - (w / 3), c, c,
				c - (w / 3), c - (h / 5), c,

				c, c - (2*h / 5), c,
				c + (w / 3), c - (h / 5), c,
				c + (w / 3), c, c,
				c + (w / 6), c + (h / 10), c,

				// line 1 back
				c, c - (h / 5), c-d,
				c, c, c - d,
				c + (w / 3), c + (h / 5), c - d,
				c + (w / 3), c + (2 * h / 5), c - d,

				c, c + (3 * h / 5), c - d,
				c - (w / 3), c + (2 * h / 5), c - d,
				c - (w / 3), c + (h / 5), c - d,
				c - (w / 6), c + (h / 10), c - d,


				// line 2 back
				c, c + (2 * h / 5), c - d,
				c, c + (h / 5), c - d,
				c - (w / 3), c, c - d,
				c - (w / 3), c - (h / 5), c - d,

				c, c - (2 * h / 5), c - d,
				c + (w / 3), c - (h / 5), c - d,
				c + (w / 3), c, c - d,
				c + (w / 6), c + (h / 10), c - d

				);

			break;
			}
			case 3: {
				float s = size;

				// Corners of the image in the pattern coordinate system
				modelPts = (cv::Mat_<float>(4, 3) <<
					0, 0, 0,
					s, 0, 0,
					s, s, 0,
					0, s, 0
					);
				break;
			}
			case 1: 
			case 4: { 
				modelPts = (Mat_<float>(8, 3) <<
					0, 0, 0,
					size, 0, 0,
					size, size, 0,
					0, size, 0,
					0, 0, -size,
					size, 0, -size,
					size, size, -size,
					0, size, -size);
				break;
			}
			case 5: 
			default:
				break;
		}

			
			modelPts.copyTo(modelPts1);

			std::vector<cv::Point2f> model2ImagePts;
			/* project model 3D points to the image. Points through the transformation matrix
			(defined by rotVec and transVec) "are transfered" from the pattern CS to the
			camera CS, and then, points are projected using camera parameters
			(camera matrix, distortion matrix) from the camera 3D CS to its image plane
			*/

			try {
				projectPoints(modelPts1, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);
			}
			catch (const cv::Exception& e) {
				std::cout << "[OpenCV Error] Case 3 failed: " << e.what() << std::endl;
				return;
			}

			switch (id) {
			case 2: {
				for (int i = 0; i < 7; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 1), Vec3b(200,200,0), 3);
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 16), color, 3);
				}
				for (int i = 8; i < 15; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 1), Vec3b(200, 200, 0), 3);
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 16), color, 3);
				}
				for (int i = 16; i < 23; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 1), Vec3b(200, 122, 0), 3);
				}
				for (int i = 24; i < 31; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 1), Vec3b(200, 122, 0), 3);
				}
				break;
			}
			case 3: {

				if (button2) {
					// Force to BGR (3 channels)
					if (patternImage.channels() == 4)
						cv::cvtColor(patternImage, patternImage, cv::COLOR_BGRA2BGR);
					
					if (button3) cv::bitwise_not(patternImage, useImage);
					else useImage = patternImage.clone();
					

					std::vector<cv::Point2f> imgPts;
					cv::projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, imgPts);

					std::vector<cv::Point2f> srcCorners = {
						{0, 0},
						{(float)useImage.cols, 0},
						{(float)useImage.cols, (float)useImage.rows},
						{0, (float)useImage.rows}
					};

					cv::Mat H = cv::getPerspectiveTransform(srcCorners, imgPts);

					cv::Mat warped;
					cv::warpPerspective(useImage, warped, H, frame.size(),
						cv::INTER_LINEAR,
						cv::BORDER_CONSTANT);

					// warped is 3-channel BGR
					for (int y = 0; y < warped.rows; ++y)
					{
						for (int x = 0; x < warped.cols; ++x)
						{
							if (x < 0 || x >= frame.cols || y < 0 || y >= frame.rows)
								continue;
							cv::Vec3b px;
							px = warped.at<cv::Vec3b>(y, x);
							

							// Skip black pixels (optional)
							if (px == cv::Vec3b(0, 0, 0))
								continue;

							frame.at<cv::Vec3b>(y, x) = px;
						}
					}
				}
				

				break;
			}
			case 1: {

				cout << rotVec << endl;
				//draw cube, or whatever
				int i;
				Scalar ncolor = (button1) ? Scalar(122, 2, 255) : color;
				for (i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i % 4), model2ImagePts.at((i + 1) % 4), ncolor, 3);
				}
				for (i = 4; i < 7; i++) {
					cv::line(frame, model2ImagePts.at(i % 8), model2ImagePts.at((i + 1) % 8), ncolor, 3);
				}
				cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), ncolor, 3);
				for (i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 4), ncolor, 3);
				}
				break;
			}
			case 4: {
				
				cout << rotVec << endl;
				//draw cube, or whatever
				int i;
				for (i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i % 4), model2ImagePts.at((i + 1) % 4), color, 3);
				}
				for (i = 4; i < 7; i++) {
					cv::line(frame, model2ImagePts.at(i % 8), model2ImagePts.at((i + 1) % 8), color, 3);
				}
				cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), color, 3);
				for (i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 4), color, 3);
				}
				break;
			}
			case 5: 
			default:
				break;
			}

			//draw the line that reflects the orientation. It indicates the bottom side of the pattern
			cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), Scalar(80, 255, 80), 3);
			model2ImagePts.clear();
		

		return;
	}
		
}