#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstring>

using namespace cv;
using namespace std;
const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
//////////////////////////////////////////////////////////
// image_captured
//////////////////////////////////////////////////////////
class image_captured {
public:
    cv::Mat image;                       // processed image with points drawn
    std::vector<cv::Point2f> points;     // detected keypoints (NOT saved to file)
    std::string filename;                // filename of the saved image

    image_captured(const cv::Mat& img,
        const std::vector<cv::Point2f>& pts,
        const std::string& file)
        : image(img.clone()), points(pts), filename(file)
    {
        cv::imwrite(filename, img);
    }
};

//////////////////////////////////////////////////////////
// calibration_data
//////////////////////////////////////////////////////////
class calibration_data {
public:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    calibration_data() {
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    }

    void saveToXML(const std::string& file) {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        fs << "camera_matrix" << cameraMatrix;
        fs << "dist_coeffs" << distCoeffs;
        fs.release();
    }
};

//////////////////////////////////////////////////////////
// camera_calibration
//////////////////////////////////////////////////////////
class camera_calibration {
private:
    std::vector<image_captured>& captures;

public:
    camera_calibration(std::vector<image_captured>& cap)
        : captures(cap) {
    }

    calibration_data performCalibration(cv::Size boardSize, float squareSize) {

        std::vector<std::vector<cv::Point3f>> objectPoints;
        std::vector<std::vector<cv::Point2f>> imagePoints;

        // Prepare object coordinates
        std::vector<cv::Point3f> obj;
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

        // For each captured image
        for (auto& cap : captures) {
            imagePoints.push_back(cap.points);
            objectPoints.push_back(obj);
        }

        calibration_data result;

        std::vector<cv::Mat> rvecs, tvecs;

        double rms = cv::calibrateCamera(objectPoints,
            imagePoints,
            captures[0].image.size(),
            result.cameraMatrix,
            result.distCoeffs,
            rvecs,
            tvecs);

        std::cout << "RMS error = " << rms << std::endl;

        return result;
    }
};

//////////////////////////////////////////////////////////
// image_corrector
//////////////////////////////////////////////////////////
class image_corrector {
private:
    calibration_data calib;

public:
    image_corrector(const calibration_data& c) : calib(c) {}

    void undistortAndShow(const cv::Mat& frame) {
        cv::Mat undist;
        cv::undistort(frame, undist, calib.cameraMatrix, calib.distCoeffs);
        cv::imshow("Camera", undist);
    }
};

//////////////////////////////////////////////////////////
// Helper: timestamp-based filename
//////////////////////////////////////////////////////////
std::string generateFilename(int counter) {
    auto t = std::time(nullptr);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif

    std::ostringstream ss;
    ss << "capture_"
        << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
        << "_" << std::setw(2) << std::setfill('0') << counter
        << ".png";

    return ss.str();
}

//////////////////////////////////////////////////////////
// MAIN PROGRAM (ACQUISITION LOOP LIVES ONLY HERE)
//////////////////////////////////////////////////////////
int main() {

    // Load config XML
    cv::FileStorage fs("config.xml", cv::FileStorage::READ);

    int board_w, board_h;
    float squareSize;

    fs["BoardSize_Width"] >> board_w;
    fs["BoardSize_Height"] >> board_h;
    fs["Square_Size"] >> squareSize;
    fs.release();


    cv::Size boardSize(board_w, board_h);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Camera not found!" << std::endl;
        return -1;
    }

    std::vector<image_captured> capturedImages;
    int successful = 0;
    int counter = 1;

    while (successful < 20) {

        //////////////////////////////
        // 1. Show live frame (1 sec)
        //////////////////////////////
        cv::Mat frame;
        for (int i = 0; i < 50; i++) {
            cap >> frame;
            if (frame.empty()) continue;
            cv::imshow("Camera", frame);
            cv::waitKey(10);
        }
        //////////////////////////////
        // 2. Next frame: detect keypoints
        //////////////////////////////
        cap >> frame;
        if (frame.empty()) continue;

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(frame, boardSize, corners);

        if (found) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            cv::drawChessboardCorners(frame, boardSize, corners, true);

            std::string file = generateFilename(counter++);
            image_captured img(frame, corners, file);
            capturedImages.push_back(img);

            successful++;
            string msg = "capturing";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 1, 2, 3, &baseLine);
            Point textOrigin(frame.cols - 2 * textSize.width - 10, frame.rows - 2 * baseLine - 10);
            Point textnext(frame.cols - 2 * textSize.width - 10, frame.rows - 3 * baseLine - 10);
            msg = format("%d/%d Undist", successful, 20);
            putText(frame, msg, textOrigin, 1, 2, GREEN, 3);

            cv::imshow("Camera", frame);
            cv::waitKey(750);
            putText(frame, "Next frame", textnext, 1, 2, GREEN, 3);
            cv::imshow("Camera", frame);
            cv::waitKey(250);

            std::cout << "Captured " << successful << " / 20" << std::endl;
        }

        else {
            cv::putText(frame, "Pattern not found", cv::Point(50, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Camera", frame);
            cv::waitKey(500);
        }
    }

    ////////////////////////////////////////////////////////
    // Perform calibration
    ////////////////////////////////////////////////////////
    cout << "calibrating....";
    camera_calibration calibManager(capturedImages);
    calibration_data data = calibManager.performCalibration(boardSize, squareSize);

    data.saveToXML("camera_parameters.xml");


    ////////////////////////////////////////////////////////
    // Show menu
    ////////////////////////////////////////////////////////
    image_corrector corrector(data);
    cv::Mat frame;
    char key = 'N';
    bool undistor = false;
    while (true) {
        cap >> frame;
        // cv::Mat menu(400, 600, CV_8UC3, cv::Scalar(50, 50, 50));
        cv::putText(frame, "Calibration Successful", cv::Point(50, 80),
            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Apply correction? (Y/N)", cv::Point(50, 160),
            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);


        if (undistor)
            corrector.undistortAndShow(frame);
        else
            cv::imshow("Camera", frame);
        key = cv::waitKey(10);
        if (key == 'Y' || key == 'y') {
            undistor = true;
        }
        if (key == 'N' || key == 'n')
            undistor = false;
        if (key == 27) break;

    }

    return 0;
}
