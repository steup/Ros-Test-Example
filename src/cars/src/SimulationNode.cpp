#include "OccupancyGridRoads.h"
#include "Simulation.h"

#include <cars/NewCar.h>

#include <ros/ros.h>

#include <memory>

using namespace std;

unique_ptr<Simulation> simPtr;

void run(const ros::TimerEvent& e) {
  simPtr->run();
}

bool createCar(cars::NewCar::Request& req, cars::NewCar::Response& res) {
  res.success = simPtr->createCar(req.numberPlate);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "road_simulation");
  ros::Duration frameRate(0.1);
  OccupancyGridRoads roads(18, 18, frameRate);
  Simulation sim(roads);
  simPtr = unique_ptr<Simulation>(&sim);
  ros::NodeHandle n;
  ros::Timer timer = n.createTimer(frameRate, &run);
  ros::ServiceServer registryServer = n.advertiseService("carRegistry", &createCar);
  while(ros::ok())
    ros::spin();
  return 0;
}
