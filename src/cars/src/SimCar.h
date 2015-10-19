#pragma once

#include "SynchronizedCar.h"
#include "ros/ros.h"

class SimCar : public SynchronizedCar{
  private:
    ros::Publisher windowPub;
    ros::Subscriber steerSub;

    void handleSteer(const cars::Steer::ConstPtr& stateMsg);

  public:
    enum class Dir {
      Up, Down, Left, Right
    } dir;
    unsigned int x, y;

    SimCar(const std::string numberPlate, Dir dir, unsigned int x, unsigned int y);
    virtual ~SimCar() {}
    virtual void windows(bool left, bool front, bool right);
};
