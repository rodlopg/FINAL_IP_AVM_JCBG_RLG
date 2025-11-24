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
	// Extra #2: imagenes para el cubo (marker 1)
    static cv::Mat faceImg1 = cv::imread("img1.jpeg", cv::IMREAD_UNCHANGED);
    static cv::Mat faceImg2 = cv::imread("img2.jpeg", cv::IMREAD_UNCHANGED);
    static cv::Mat faceImg3 = cv::imread("img3.png", cv::IMREAD_UNCHANGED);
    static cv::Mat faceImg4 = cv::imread("img4.jpg", cv::IMREAD_UNCHANGED);
    static cv::Mat faceImg5 = cv::imread("img5.png", cv::IMREAD_UNCHANGED);
	// Extra #4: dígitos 0–9 en ciclo (marker 7)
	static int currentDigit = 0;
	static double lastDigitChangeTime = 0.0; // en segundos


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

		// Ayuda para extra #2
		auto overlayImageOnFace = [&](const cv::Mat& imgIn, const std::vector<cv::Point3f>& face3D)
		{
			if (imgIn.empty()) return;

			cv::Mat img;
			if (imgIn.channels() == 4)
				cv::cvtColor(imgIn, img, cv::COLOR_BGRA2BGR);
			else
				img = imgIn;

			// Proyectar los 4 puntos 3D de la cara al plano de la imagen
			std::vector<cv::Point2f> face2D;
			cv::projectPoints(face3D, rotVec, transVec, camMatrix, distMatrix, face2D);

			if (face2D.size() != 4) return;

			std::vector<cv::Point2f> srcCorners = {
				{0.f, 0.f},
				{(float)img.cols, 0.f},
				{(float)img.cols, (float)img.rows},
				{0.f, (float)img.rows}
			};

			cv::Mat H = cv::getPerspectiveTransform(srcCorners, face2D);

			cv::Mat warped;
			cv::warpPerspective(img, warped, H, frame.size(),
								cv::INTER_LINEAR,
								cv::BORDER_CONSTANT);

			// Copiar píxeles no negros al frame
			for (int y = 0; y < warped.rows; ++y)
			{
				if (y < 0 || y >= frame.rows) continue;
				for (int x = 0; x < warped.cols; ++x)
				{
					if (x < 0 || x >= frame.cols) continue;

					cv::Vec3b px = warped.at<cv::Vec3b>(y, x);
					if (px == cv::Vec3b(0, 0, 0))
						continue;

					frame.at<cv::Vec3b>(y, x) = px;
				}
			}
		};

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
			//Changed some colors
			case 1: { color = Scalar(255, 255, 10); break; }
			case 2:	{ color = Scalar(0, 0, 255); break;	}
			case 3:	{ color = Scalar(255, 255, 0); break;	}
			case 4:	{ color = Scalar(255, 10, 255); break;	} 
			case 5: { color = Scalar(255, 100, 90); break; } //Added and not used
			default:
				break;

		}

		Mat modelPts;

		//Added the switch statement
		switch (id) {
		case 2: {
			//Cool S ADDED THIS WHOLE CASE
			float w = size;          // width of body
			float h = size * 1.6f;   // body height
			float d = size * 0.4f;   // depth (thickness)
			float c = 50;

			modelPts = (Mat_<float>(32, 3) <<
				// line 1 front
				//Added
				c, c - (h/5), c,
				c, c, c,
				c + (w / 3), c + (h / 5), c,
				c + (w / 3), c + (2*h / 5), c,

				c, c + (3 * h / 5), c,
				c - (w / 3), c + (2 * h / 5), c,
				c - (w / 3), c + (h / 5), c,
				c - (w / 6), c + (h / 10), c,


				// line 2 front
				//Added
				c, c + (2 * h / 5), c,
				c, c + (h / 5), c,
				c - (w / 3), c, c,
				c - (w / 3), c - (h / 5), c,

				c, c - (2*h / 5), c,
				c + (w / 3), c - (h / 5), c,
				c + (w / 3), c, c,
				c + (w / 6), c + (h / 10), c,

				// line 1 back
				//Added
				c, c - (h / 5), c-d,
				c, c, c - d,
				c + (w / 3), c + (h / 5), c - d,
				c + (w / 3), c + (2 * h / 5), c - d,

				c, c + (3 * h / 5), c - d,
				c - (w / 3), c + (2 * h / 5), c - d,
				c - (w / 3), c + (h / 5), c - d,
				c - (w / 6), c + (h / 10), c - d,


				// line 2 back
				//Added
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
				//Added this to put the corners of the image to draw
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
				//Moved the cube here
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
			case 7: {
				modelPts = (Mat_<float>(4, 3) <<
					0,      0,      0,
					size,   0,      0,
					size,   size,   0,
					0,      size,   0 );
				break;
			}
			
			case 6: {
				// Same cube as 1 & 4
				modelPts = (cv::Mat_<float>(8, 3) <<
					0,     0,      0,
					size,  0,      0,
					size,  size,   0,
					0,     size,   0,
					0,     0,     -size,
					size,  0,     -size,
					size,  size,  -size,
					0,     size,  -size);
				break;
			}

			case 5: {
				// Extra #3 10 vertex

				float s = (float)size;
				float cx = s * 0.5f;
				float cy = s * 0.5f;
				float r = s * 0.35f;      
				float zTop = 0.0f;        
				float zBottom = -s;      

				modelPts = cv::Mat(10, 3, CV_32F);
				for (int i = 0; i < 5; ++i) {
					float angle = 2.0f * (float)CV_PI * i / 5.0f;

					float x = cx + r * std::cos(angle);
					float y = cy + r * std::sin(angle);

					// Superior vertex (0–4)
					modelPts.at<float>(i, 0) = x;
					modelPts.at<float>(i, 1) = y;
					modelPts.at<float>(i, 2) = zTop;

					// Inferior vertex (5–9)
					modelPts.at<float>(i + 5, 0) = x;
					modelPts.at<float>(i + 5, 1) = y;
					modelPts.at<float>(i + 5, 2) = zBottom;
				}
				break;
			}


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
			//Added this trycatch
			try {
				projectPoints(modelPts1, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);
			}
			catch (const cv::Exception& e) {
				std::cout << "[OpenCV Error] Case 3 failed: " << e.what() << std::endl;
				return;
			}

			//Added this wole switch case to manage drawing of figures
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
				//Added this to manage cases for the image and the negation of the image with button2 (to show image) and button 3 (to negate it)
				if (button2) {
					// Force to BGR (3 channels)
					if (patternImage.channels() == 4)
						cv::cvtColor(patternImage, patternImage, cv::COLOR_BGRA2BGR); //Making sure the image has 3 channels (casting)
					
					if (button3) cv::bitwise_not(patternImage, useImage); //Negating image
					else useImage = patternImage.clone();
					

					std::vector<cv::Point2f> imgPts;
					cv::projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, imgPts);

					//Setting coordinates relative to image size and transform it later
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
					//Draw image
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
				// Extra #2
				cout << rotVec << endl;
				int i;
				Scalar ncolor = (button1) ? Scalar(122, 2, 255) : color; // cambia color si button1 activo
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

				// Define the faces
				float s = (float)size;

				// Frontal face  (z = 0)
				std::vector<cv::Point3f> faceFront = {
					{0.f, 0.f, 0.f},
					{s, 0.f, 0.f},
					{s, s, 0.f},
					{0.f, s, 0.f}
				};

				// Back face (z = -s)
				std::vector<cv::Point3f> faceBack = {
					{0.f, 0.f, -s},
					{s, 0.f, -s},
					{s, s, -s},
					{0.f, s, -s}
				};

				// Left face (x = 0)
				std::vector<cv::Point3f> faceLeft = {
					{0.f, 0.f, 0.f},
					{0.f, 0.f, -s},
					{0.f, s, -s},
					{0.f, s, 0.f}
				};

				// Right face (x = s)
				std::vector<cv::Point3f> faceRight = {
					{s, 0.f, 0.f},
					{s, 0.f, -s},
					{s, s, -s},
					{s, s, 0.f}
				};

				// Top Face (y = 0)
				std::vector<cv::Point3f> faceTop = {
					{0.f, 0.f, 0.f},
					{s, 0.f, 0.f},
					{s, 0.f, -s},
					{0.f, 0.f, -s}
				};

				// The image on each face
				overlayImageOnFace(faceImg1, faceFront); 
				overlayImageOnFace(faceImg2, faceBack);  
				overlayImageOnFace(faceImg3, faceLeft);  
				overlayImageOnFace(faceImg4, faceRight); 
				overlayImageOnFace(faceImg5, faceTop);   

				break;
			}
			case 4: {
				
				cout << rotVec << endl;
				//Draw cube for marker 4, color dont change
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
			case 7: {
				// Extra #4
				// Update by time
				double now = (double)cv::getTickCount() / cv::getTickFrequency(); 
				if (now - lastDigitChangeTime >= 1.0) { // more than a second
					currentDigit = (currentDigit + 1) % 10;
					lastDigitChangeTime = now;
				}

				// "Display" by coordinates
				float s = (float)size;
				float x1 = 0.2f * s;
				float x2 = 0.8f * s;
				float y1 = 0.1f * s;
				float y2 = 0.9f * s;
				float yMid = 0.5f * (y1 + y2);
				float zFront = 0.0f;         
				float zBack  = -0.3f * s;    

				cv::Point2f seg2D_start[7] = {
					{x1, y1},      // A top
					{x2, y1},      // B top-right
					{x2, yMid},    // C bottom-right (parte alta)
					{x1, y2},      // D bottom
					{x1, yMid},    // E bottom-left
					{x1, y1},      // F top-left
					{x1, yMid}     // G middle
				};

				cv::Point2f seg2D_end[7] = {
					{x2, y1},      // A
					{x2, yMid},    // B
					{x2, y2},      // C
					{x2, y2},      // D
					{x1, y2},      // E
					{x1, yMid},    // F
					{x2, yMid}     // G
				};

				static const bool digitSeg[10][7] = {
					// A,   B,   C,   D,   E,   F,   G
					{ true, true, true, true, true, true, false }, // 0
					{ false,true, true, false,false,false,false }, // 1
					{ true, true, false,true, true, false, true  }, // 2
					{ true, true, true, true, false,false, true  }, // 3
					{ false,true, true, false,false, true,  true }, // 4
					{ true, false,true, true, false,true,  true }, // 5
					{ true, false,true, true, true, true,  true }, // 6
					{ true, true, true, false,false,false,false }, // 7
					{ true, true, true, true, true, true,  true }, // 8
					{ true, true, true, true, false,true,  true }  // 9
				};

				std::vector<cv::Point3f> digit3D;
				for (int sIdx = 0; sIdx < 7; ++sIdx) {
					if (!digitSeg[currentDigit][sIdx]) continue;

					cv::Point2f a = seg2D_start[sIdx];
					cv::Point2f b = seg2D_end[sIdx];

					cv::Point2f dir = b - a;
					float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
					if (len < 1e-3f) continue;
					cv::Point2f n(-dir.y / len, dir.x / len); 

					float thickness = 0.04f * s;

					cv::Point2f a1 = a + n * thickness;
					cv::Point2f a2 = a - n * thickness;
					cv::Point2f b1 = b + n * thickness;
					cv::Point2f b2 = b - n * thickness;

					// Front (zFront) & Back (zBack)
					cv::Point3f p1(a1.x, a1.y, zFront);
					cv::Point3f p2(b1.x, b1.y, zFront);
					cv::Point3f p3(b2.x, b2.y, zFront);
					cv::Point3f p4(a2.x, a2.y, zFront);

					cv::Point3f p5(a1.x, a1.y, zBack);
					cv::Point3f p6(b1.x, b1.y, zBack);
					cv::Point3f p7(b2.x, b2.y, zBack);
					cv::Point3f p8(a2.x, a2.y, zBack);

					digit3D.push_back(p1);
					digit3D.push_back(p2);
					digit3D.push_back(p3);
					digit3D.push_back(p4);
					digit3D.push_back(p5);
					digit3D.push_back(p6);
					digit3D.push_back(p7);
					digit3D.push_back(p8);
				}

				if (digit3D.empty()) break;

				std::vector<cv::Point2f> digit2D;
				try {
					cv::projectPoints(digit3D, rotVec, transVec, camMatrix, distMatrix, digit2D);
				}
				catch (const cv::Exception& e) {
					std::cout << "[OpenCV Error] case 7 (digits 3D) failed: " << e.what() << std::endl;
					break;
				}

				for (size_t i = 0; i + 7 < digit2D.size(); i += 8) {
					// Front
					cv::line(frame, digit2D[i + 0], digit2D[i + 1], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 1], digit2D[i + 2], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 2], digit2D[i + 3], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 3], digit2D[i + 0], Scalar(0, 255, 255), 2);

					// Back
					cv::line(frame, digit2D[i + 4], digit2D[i + 5], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 5], digit2D[i + 6], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 6], digit2D[i + 7], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 7], digit2D[i + 4], Scalar(0, 255, 255), 2);

					cv::line(frame, digit2D[i + 0], digit2D[i + 4], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 1], digit2D[i + 5], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 2], digit2D[i + 6], Scalar(0, 255, 255), 2);
					cv::line(frame, digit2D[i + 3], digit2D[i + 7], Scalar(0, 255, 255), 2);
				}

				break;
			}

			case 5: {
				if (model2ImagePts.size() < 10) break;

				cv::Scalar col(0, 200, 255);

				for (int i = 0; i < 5; ++i) {
					cv::Point2f p1 = model2ImagePts[i];
					cv::Point2f p2 = model2ImagePts[(i + 1) % 5];
					cv::line(frame, p1, p2, col, 3);
				}

				for (int i = 0; i < 5; ++i) {
					cv::Point2f p1 = model2ImagePts[5 + i];
					cv::Point2f p2 = model2ImagePts[5 + ((i + 1) % 5)];
					cv::line(frame, p1, p2, col, 3);
				}

				for (int i = 0; i < 5; ++i) {
					cv::Point2f p1 = model2ImagePts[i];
					cv::Point2f p2 = model2ImagePts[i + 5];
					cv::line(frame, p1, p2, col, 3);
				}

				break;
			}

			case 6: {
				// Extra #3 B imagen inside a cube

				// Draw a cube
				if (model2ImagePts.size() < 8) break;

				cv::Scalar cubeColor(255, 0, 120);
				for (int i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i % 4), model2ImagePts.at((i + 1) % 4), cubeColor, 3);
				}
				for (int i = 4; i < 7; i++) {
					cv::line(frame, model2ImagePts.at(i % 8), model2ImagePts.at((i + 1) % 8), cubeColor, 3);
				}
				cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), cubeColor, 3);
				for (int i = 0; i < 4; i++) {
					cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 4), cubeColor, 3);
				}

				double t = (double)cv::getTickCount() / cv::getTickFrequency(); 
				double speed = 0.25; 
				double phase = std::fmod(t * speed, 1.0); 

				float s = (float)size;
				float h = (float)(phase * s); 

				std::vector<cv::Point3f> innerPlane = {
					{0.f,    h,  0.f},
					{s,      h,  0.f},
					{s,      h, -s },
					{0.f,    h, -s }
				};

				overlayImageOnFace(patternImage, innerPlane);

				break;
			}


			default:
				break;
			}

			//draw the line that reflects the orientation. It indicates the bottom side of the pattern
			cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), Scalar(80, 255, 80), 3);
			model2ImagePts.clear();
		

		return;
	}
		
}