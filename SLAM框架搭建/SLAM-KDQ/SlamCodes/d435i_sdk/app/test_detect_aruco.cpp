#include  <opencv2/aruco.hpp>
#include "d435i.hpp"

int main(int argc,char **argv) {
  D435I d435i;
  d435i.start();
  while(1) {
    double timestamp;
    cv::Mat leftImg,rightImg;
    if (!d435i.getInfraredImages(timestamp,leftImg,rightImg)) {
      continue;
    }
    if (!leftImg.empty()) {
      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      cv::aruco::detectMarkers(leftImg, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        // if at least one marker detected
      if (markerIds.size() > 0)
        cv::aruco::drawDetectedMarkers(leftImg, markerCorners, markerIds);
      cv::imshow("detect aruco",leftImg);

    }
    cv::waitKey(1);
  }
  return 0;
}