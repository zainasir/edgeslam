#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <string>

int main() {
  const std::string RTSP_URL = "rtsp://anafi.local/live";

#if WIN32
  _putenv_s("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp");
#else
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp", 1);
#endif

  cv::Mat frame;
  cv::VideoCapture cap(RTSP_URL, cv::CAP_FFMPEG);

  if (!cap.isOpened()) {
    std::cout << "Cannot open RTSP stream" << std::endl;
    return -1;
  }

  while (true) {
    cap >> frame;
    imshow("RTSP stream", frame);

    if (waitKey(1) == 27) {
      break;
    }
  }

  cap.release();
  destroyAllWindows();

  return 0;
}
