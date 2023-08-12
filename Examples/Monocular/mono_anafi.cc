#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <string>

using namespace std;

int main(int argc, char **argv) {
  // Argument Checking
  if (argc < 5) {
    cerr << endl << "Client Usage: ./test VOC_PATH SETTINGS_PATH VIDEO_PATH RUN_TYPE(client|server)" << endl;
    return -1;
  }

  // Open video file
  std::string videoPath = argv[3];
  cv::VideoCapture videoCap(videoPath);

  if (!videoCap.isOpened()) {
    cerr << endl << "Error opening video file!" << endl;
    return -1;
  }
  
  // Check runtype and convert to lowercase
  std::string RunType(argv[4]);
  std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);

  // Client
  if (RunType.compare("client") == 0) {
    
    // Set up SLAM system
    //ORB_SLAM2::System SLAM(argv[1], argv[2], RunType, ORB_SLAM2::System::MONOCULAR, true);

    while (1) {
      cv::Mat currFrame;
      videoCap >> currFrame;

      if (currFrame.empty()) {
	break;
      }

      else {
	imshow("Video", currFrame);
      }

      char interruptKey = (char) cv::waitKey(25);
      if (interruptKey == 27) {
	break;
      }
    }

    videoCap.release();
    cv::destroyAllWindows();
  }

  return 0;
}
