#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "d435i.hpp"
#include <iostream>
#include <string>
namespace {
const char* about = "A tutorial code on charuco board creation and detection of charuco board with and without camera caliberation";
const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to detect charuco board without camera calibration;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}
void createBoard();
void detectCharucoBoardWithCalibrationPose();
void detectCharucoBoardWithoutCalibration();
static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(8, 5, 0.03f, 0.0225f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(800, 800), boardImage, 10, 1);
    cv::imwrite("BoardImage.jpg", boardImage);
}
void detectCharucoBoardWithCalibrationPose()
{
    D435I d435i;
    d435i.start();
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(8, 5, 0.03f, 0.0225f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        while (1) {
            double timestamp;
            cv::Mat leftImg,rightImg;
            if (!d435i.getInfraredImages(timestamp,leftImg,rightImg)) {
              continue;
            }
            cv::Mat image;
            cv::Mat imageCopy;
            leftImg.copyTo(image);
            image.copyTo(imageCopy);
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    cv::Vec3d rvec, tvec;
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
        }
    }
}
void detectCharucoBoardWithoutCalibration()
{
    D435I d435i;
    d435i.start();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(8, 5, 0.03f, 0.0225f, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    while (1) {
        double timestamp;
        cv::Mat leftImg, rightImg;
        if (!d435i.getInfraredImages(timestamp,leftImg, rightImg)) {
          continue;
        }
        cv::Mat image, imageCopy;
        leftImg.copyTo(image);
        cvtColor(image,imageCopy,cv::COLOR_GRAY2BGR);
        
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            //cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds);
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
        }
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(30);
        if (key == 27)
            break;
    }
}
int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (argc < 2) {
        parser.printMessage();
        return 0;
    }
    int choose = parser.get<int>("c");
    switch (choose) {
    case 1:
        createBoard();
        std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
        break;
    case 2:
        detectCharucoBoardWithoutCalibration();
        break;
    case 3:
        detectCharucoBoardWithCalibrationPose();
        break;
    default:
        break;
    }
    return 0;
}