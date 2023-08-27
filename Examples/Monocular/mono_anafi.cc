#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <System.h>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv) {
  // Argument Checking
  if (argc < 4) {
    cerr << endl << "Client Usage: ./mono_anafi VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) VIDEO_PATH" << endl;
    cerr << endl << "Client Usage: ./mono_anafi VOC_PATH SETTINGS_PATH RUN_TYPE(client|server)" << endl;
    return -1;
  }

  // Check runtype and convert to lowercase
  std::string RunType(argv[3]);
  std::transform(RunType.begin(), RunType.end(), RunType.begin(), ::tolower);

  // Client
  if (RunType.compare("client") == 0) {

    // Open video file
    std::string videoPath = argv[4];
    cv::VideoCapture videoCap(videoPath);

    if (!videoCap.isOpened()) {
      cerr << endl << "Error opening video file!" << endl;
      return -1;
    }

    // Set up SLAM system
    ORB_SLAM2::System SLAM(argv[1], argv[2], RunType, ORB_SLAM2::System::MONOCULAR, true);
    cout << endl << "--------" << endl;
    cout << "Start processing sequence ..." << endl;

    #ifdef COMPILEDWITHC11
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    #else
    chrono::monotonic_clock::time_point t1 = chrono::monotonic_clock::now();
    #endif
    
    // Main loop
    while (true) {
      cv::Mat frame;
      videoCap >> frame;
      
      if (frame.empty()) {
	break;
      }

      #ifdef COMPILEDWITHC11
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      #else
      chrono::monotonic_clock::time_point t2 = chrono::monotonic_clock::now();
      #endif

      SLAM.TrackMonocular(frame, chrono::duration_cast<chrono::duration<double>>(t2 - t1).count());
    }

    // Split shutdown between client and server
    SLAM.ClientShutdown();
  }

  else if (RunType.compare("server") == 0) {
    // Create SLAM system
    ORB_SLAM2::System SLAM(argv[1], argv[2], RunType, ORB_SLAM2::System::MONOCULAR, true);

    // Stop all threads
    SLAM.ServerShutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory,txt");
  }

  else {
    cerr << endl << "Client Usage: ./mono_anafi VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) VIDEO_PATH" << endl;
    cerr << endl << "Server Usage: ./mono_anafi VOC_PATH SETTINGS_PATH RUN_TYPE(client|server) VIDEO_PATH" << endl;
    return -1;
  }

  return 0;
}
