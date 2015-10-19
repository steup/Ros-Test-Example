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
  while(true) {
    try {
      SimpleDriver driver(plate);
      while(ros::ok())
        ros::spin();
    }catch(runtime_error&){
      ROS_INFO_STREAM("Simulation not running (yet)");
    }
    catch(invalid_argument&){
      throw;
      break;
    }
  }
  return 0;
}
