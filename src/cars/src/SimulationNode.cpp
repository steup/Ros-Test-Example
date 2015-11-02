#include "OccupancyGridRoads.h"
#include "Simulation.h"
#include "SimCar.h"

#include <cars/NewCar.h>

#include <ros/ros.h>

#include <memory>

using namespace std;

using Sim = Simulation<SimCar>;
using SimPtr = unique_ptr<Sim>;

SimPtr simPtr;

void run(const ros::TimerEvent& e) {
  simPtr->run();
}

bool createCar(cars::NewCar::Request& req, cars::NewCar::Response& res) {
  res.success = simPtr->createCar(req.numberPlate);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "road_simulation");
  double frameRate;
  if(!ros::param::has("~frameRate")){
    ROS_ERROR_STREAM("No simulation frame rate specified!");
    return -1;
  }
  if(!ros::param::get("~frameRate", frameRate)){
    ROS_ERROR_STREAM("Invalid simulation frame rate specified!");
    return -2;
  }
  OccupancyGridRoads roads(18, 18, ros::Duration(frameRate));
  simPtr = SimPtr(new Sim(roads));
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(ros::Duration(frameRate), &run);
  ros::ServiceServer registryServer = n.advertiseService("carRegistry", &createCar);
  while(ros::ok())
    ros::spin();
  return 0;
}
