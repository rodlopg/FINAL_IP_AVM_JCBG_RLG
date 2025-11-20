#include "pattern.h"
#include <iostream>
#include "patterndetector.h"


using namespace cv;
using namespace std;

namespace ARma {

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

	

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix)
	{

		Scalar color = Scalar(255, 255, 255);
		//model 3D points: they must be projected to the image plane
		int op = 0;
		Mat modelPts1;

		switch (id) {
		case 1: { color = Scalar(255, 0, 255); break; }
		case 2:	{color = Scalar(0, 0, 255); break;	}
		case 3:	{color = Scalar(255, 255, 0); break;	}
		case 4:	{color = Scalar(255, 0, 0); break;	}
		default:
			break;
		}
			Mat modelPts = (Mat_<float>(8, 3) <<
				0, 0, 0,
				size, 0, 0,
				size, size, 0,
				0, size, 0,
				0, 0, -size,
				size, 0, -size,
				size, size, -size,
				0, size, -size);
			modelPts.copyTo(modelPts1);

			std::vector<cv::Point2f> model2ImagePts;
			/* project model 3D points to the image. Points through the transformation matrix
			(defined by rotVec and transVec) "are transfered" from the pattern CS to the
			camera CS, and then, points are projected using camera parameters
			(camera matrix, distortion matrix) from the camera 3D CS to its image plane
			*/
			projectPoints(modelPts1, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);
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

			//draw the line that reflects the orientation. It indicates the bottom side of the pattern
			cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), Scalar(80, 255, 80), 3);
			model2ImagePts.clear();
		

		return;
	}
		
}