#pragma once

#include "SynchronizedCar.h"
#include <ros/ros.h>

class Car : public SynchronizedCar {
  private:
    ros::Publisher steerPub;
    ros::Subscriber windowSub;
    void handleWindows(const cars::Windows::ConstPtr& stateMsg);
  protected:
    virtual void onWindowUpdate() {}
  public:
   Car(const std::string& numberPlate); 
   virtual ~Car() {}
   virtual void steer(Steering cmd);
   using SynchronizedCar::steer;
};
