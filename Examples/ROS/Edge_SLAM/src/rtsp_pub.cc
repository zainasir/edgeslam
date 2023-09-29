#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "../../../include/System.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtsp_pub");
  ross::start();
  ros::NodeHandle nodeHandler;

  std::string rtspPath = "rtsp://192.168.53.1/live";

  // Enable udp transport mode for opencv to detect the rtsp stream
  #if WIN32
  _putenv_s("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp");
  #else
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp", 1);
  #endif

  // Start an opencv video capture
  cv::VideoCapture videoCap(rtspPath);

  if (!videoCap.isOpened())
  {
    cerr << endl << "Failed to open RTSP stream!" << endl;
    ros::shutdown();
    return 1;
  }

  image_transport::ImageTransport imageTransporter(nodeHandler);
  image_transport::Publisher pub_frame = imageTransporter.advertise("camera", 1);

  cv::Mat frame;
  sensore_msgs::ImagePtr imageMsg;
  ros::Rate loop_rate(10);

  while (nodeHandler.ok())
  {
    videoCap >> frame;

    if (frame.empty())
    {
      cerr << endl << "Empty frame received. Shutting down stream!" << endl;
      ros::shutdown();
      return 1;
    }

    else
    {
      imshow("Video", frame);
    }

    char interruptKey = (char) cv::waitKey(25);
    if (interruptKey == 27)
    {
      ros::shutdown();
      return 1;
    }
  }
}
