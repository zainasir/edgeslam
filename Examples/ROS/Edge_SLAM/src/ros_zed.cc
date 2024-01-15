#include <iostream>
#include <string>
#include <ros/ros.h>
#include "../../../include/System.h"
#include <sl/Camera.hpp>

using namespace sl;

int main(int argc, char **argv)
{
  cout << "Hello from ZED\n";

  Camera zed;

  InitParameters init_params;
  init_params.depth_mode = DEPTH_MODE::ULTRA;
  init_params.coordinate_units = UNIT::MILLIMETER;

  // Open the camera
  auto returned_state = zed.open(init_params);
  if (returned_state != ERROR_CODE::SUCCESS) {
    cout << "Error " << returned_state << ", exit program." << endl;
    return EXIT_FAILURE;
  }

  int i = 0;
  sl::Mat image, depth, point_cloud;

  while (i < 50) {
    if (zed.grab() == ERROR_CODE::SUCCESS) {
      zed.retrieveImage(image, VIEW::LEFT);
      zed.retrieveMeasure(depth, MEASURE::DEPTH);
      zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

      int x = image.getWidth() / 2;
      int y = image.getHeight() / 2;
      sl::float4 point_cloud_value;
      point_cloud.getValue(x, y, &point_cloud_value);

      if(std::isfinite(point_cloud_value.z)){
                float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                cout<<"Distance to Camera at {"<<x<<";"<<y<<"}: "<<distance<<"mm"<<endl;
      } else {
	cout<<"The Distance can not be computed at {"<<x<<";"<<y<<"}"<<endl;
      }

      i++;
    }
  }
  
  zed.close();
  return EXIT_SUCCESS;

}
