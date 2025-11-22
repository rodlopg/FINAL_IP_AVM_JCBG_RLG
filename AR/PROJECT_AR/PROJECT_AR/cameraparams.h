#include <opencv2/core/core.hpp>
#include <opencv2/core/persistence.hpp>
#include<string>

using namespace cv;
using namespace std;

#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H
Mat cameraMatrix ;
Mat distortions;
Size calibratedImageSize(1,1);

//static bool readCameraMatrix(const string& filename);

string nameIntri = "cam_intrinsic.xml";
string namedistor = "distortion.xml";
string filenamee = "camera.yml";
//static bool readCameraMatrix(const string& filename);

bool readCameraMatrix(const string& filename)
{

	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		cerr << "failed to open " << filename << endl;

		return false;
	}


	fs["camera_matrix"] >> cameraMatrix;
	fs["dist_coeffs"] >> distortions;
	
	return true;
}


#endif


