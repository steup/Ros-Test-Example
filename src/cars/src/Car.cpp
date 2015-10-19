#include "Car.h"
#include <cars/NewCar.h>
#include <stdexcept>

using namespace std;

Car::Car(const string& numberPlate) : SynchronizedCar(numberPlate) {
  ros::NodeHandle n;
  
  steerPub =n.advertise<cars::Steer>(string("cars/")+numberPlate+"/steer", 1000);
  windowSub=n.subscribe(string("cars/")+numberPlate+"/windows", 1000, &Car::handleWindows, this);
  
  ros::ServiceClient srv = ros::NodeHandle().serviceClient<cars::NewCar>("carRegistry");
  cars::NewCar arg;
  arg.request.numberPlate = this->numberPlate();
  if (!srv.call(arg))
    throw runtime_error("Service call failed");
  if (!arg.response.success)
    throw logic_error("Cannot spawn car");
}

void Car::steer(Car::Steering cmd){
  SynchronizedCar::steer(cmd);
  steerPub.publish(mSteer);
}

void Car::handleWindows(const cars::Windows::ConstPtr& msg) {
  mWindows = *msg;
  onWindowUpdate();
}
