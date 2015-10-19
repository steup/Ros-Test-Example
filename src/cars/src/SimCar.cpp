#include "SimCar.h"

using namespace std;

SimCar::SimCar(const std::string numberPlate, SimCar::Dir dir, unsigned int x, unsigned int y)
  : SynchronizedCar(numberPlate), dir(dir), x(x), y(y) {
  ros::NodeHandle n;
  
  windowPub=n.advertise<cars::Windows>(string("cars/")+numberPlate+"/windows", 1000);
  steerSub=n.subscribe(string("cars/")+numberPlate+"/steer", 1000, &SimCar::handleSteer, this);
}

void SimCar::windows(bool left, bool front, bool right){
  SynchronizedCar::windows(left, front, right);
  windowPub.publish(mWindows);
}

void SimCar::handleSteer(const cars::Steer::ConstPtr& msg) {
  mSteer = *msg;
}
