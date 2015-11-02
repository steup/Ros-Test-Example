#pragma once

#include "SimCarInterface.h"
#include "ros/ros.h"

class SimCar : public SimCarInterface{
  private:
    ros::Publisher windowPub;
    ros::Subscriber steerSub;

    void handleSteer(const cars::Steer::ConstPtr& stateMsg);

  public:
    SimCar(const std::string numberPlate, Dir dir, unsigned int x, unsigned int y);
    virtual ~SimCar() {}
    virtual void windows(bool left, bool front, bool right);
    void dir(Dir dir) { mDir = dir; }
    void x(unsigned int x) { mX = x; }
    void y(unsigned int y) { mY = y; }
    using SimCarInterface::dir;
    using SimCarInterface::x;
    using SimCarInterface::y;
};
