#include "SimpleDriver.h"

#include <string>
#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_driver", ros::init_options::AnonymousName);
  string plate;
  if(!ros::param::has("~numberPlate")) {
    ROS_ERROR("Car is missing its number plate!");
    return -1;
  }
  if(!ros::param::get("~numberPlate", plate)) {
    ROS_ERROR("Car has invalid number plate!");
    return -2;
  }
  SimpleDriver driver(plate);
  while(ros::ok())
    ros::spin();
  return 0;
}
