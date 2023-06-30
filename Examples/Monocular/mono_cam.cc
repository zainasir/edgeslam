#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <string>

using namespace std;

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl << "Client Usage: ./mono_cam VOC_PATH SETTINGS_PATH client" << endl;
    cerr << endl << "Server Usage: ./mono_cam VOC_PATH SETTINGS_PATH server" << endl;
  }

  // Check run type and conver to lowercase
  std::string RunType(argv[3]);
  std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);

  // Run edgeslam for the client
  if (RunType.compare("client") == 0) {
    // Open webcam and set 'q' as quit button
    cv::Mat image;
    cv::namedWindow("Window", CV_WINDOW_AUTOSIZE);
    cv::VideoCapture v_cap(0);

    if (!v_cap.isOpened()) {
      cerr << endl << "Cannot open camera" << endl;
    }

    while (true) {
      v_cap >> image;
      cv::imshow("Window", image);

      if (cv::waitKey(10) >= 0) {
	break;
      }
    }
  }
}
