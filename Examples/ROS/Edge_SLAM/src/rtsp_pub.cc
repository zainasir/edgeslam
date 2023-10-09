#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "../../../include/System.h"

int skipFrame = 3;
std::string rtspPath = "rtsp://192.168.53.1/live";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtsp_pub");
  ros::start();
  ros::NodeHandle nodeHandler;

  std::string rtspPath = "rtsp://192.168.53.1/live";

  // Enable udp transport mode for opencv to detect the rtsp stream
  #if WIN32
  _putenv_s("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp");
  #else
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;udp", 1);
  #endif

  cv::VideoCapture videoCap(rtspPath);

  if (!videoCap.isOpened()) {
    cerr << endl << "Failed to open RTSP stream!" << endl;
    ros::shutdown();
    return 1;
  }

  // Limit buffer size to avoid jittering
  videoCap.set(cv::CAP_PROP_BUFFERSIZE, 10);
  
  image_transport::ImageTransport imageTransporter(nodeHandler);
  image_transport::Publisher pubFrame = imageTransporter.advertise("camera", 1);

  cv::Mat frame;
  sensor_msgs::ImagePtr imageMsg;
  ros::Rate loop_rate(10);

  int frameCount = 0;
  while (nodeHandler.ok()) {
    videoCap >> frame;

    if (frame.empty()) {
      cerr << endl << "Empty frame received. Shutting down stream!" << endl;
      ros::shutdown();
      return 1;
    }

    else if (frameCount++ % skipFrame == 0) {
      imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pubFrame.publish(imageMsg);
    }
  }

  videoCap.release();
  ros::shutdown();
  return 1;
}


