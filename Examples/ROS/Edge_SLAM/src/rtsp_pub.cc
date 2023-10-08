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
queue<cv::Mat> *frameQueue = new queue<cv::Mat>();

void ReceiveFrame()
{
  cv::VideoCapture videoCap(rtspPath);

  if (!videoCap.isOpened()) {
    cerr << endl << "Failed to open RTSP stream!" << endl;
    return;
  }

  cv::Mat frame;
  int count = 0;
  
  while (true) {
    videoCap >> frame;

    if (frame.empty()) {
      cerr << endl << "Empty frame received. Shutting down stream!" << endl;
      return;
    }

    else {
      cout << "Adding frame: " << count << ". Queue size: " << frameQueue -> size() << endl;
      frameQueue -> push(frame);
      count++;
      //imshow("RTSP Stream", frame);
    }

    char interruptKey = (char) cv::waitKey(25);
    if (interruptKey == 27) {
      return;
    }
  }
}

void DisplayFrame()
{
  while (true) {
    if (frameQueue -> size() > 0) {
      cv::Mat currFrame = frameQueue -> front();
      cout << "Displayed frame: " << currFrame.size() << endl;
      frameQueue -> pop();
    }
  }
}

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

  /*
  std::thread receiveThread(ReceiveFrame);
  std::thread displayThread(DisplayFrame);
  receiveThread.join();
  displayThread.join();

  ros::shutdown();
  return 1;
  */
  
  cv::VideoCapture videoCap(rtspPath);

  if (!videoCap.isOpened()) {
    cerr << endl << "Failed to open RTSP stream!" << endl;
    ros::shutdown();
    return 1;
  }
  
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
      cout << "Found frame: " << frameCount << endl;
      imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pubFrame.publish(imageMsg);
      cv::waitKey(1);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  videoCap.release();
  ros::shutdown();
  return 1;
}


