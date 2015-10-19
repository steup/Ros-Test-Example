#include "SynchronizedCar.h"

using namespace std;

int32_t SynchronizedCar::steerToMsg(SynchronizedCar::Steering cmd) {
  switch(cmd){
    case(Steering::Straight): return cars::Steer::Straight;
    case(Steering::Left)    : return cars::Steer::Left;
    case(Steering::Right)   : return cars::Steer::Right;
    default                 : return 0;
 }
}

SynchronizedCar::Steering SynchronizedCar::msgToSteer(int32_t cmd) {
  switch(cmd){
    case(cars::Steer::Straight): return Steering::Straight;
    case(cars::Steer::Left)    : return Steering::Left;
    case(cars::Steer::Right)   : return Steering::Right;
    default                    : return Steering::Straight;
 }
}

SynchronizedCar::SynchronizedCar(const std::string numberPlate)
  : CarInterface(numberPlate) {
  mSteer.cmd = cars::Steer::Straight;
  mWindows.left = false;
  mWindows.right = false;
  mWindows.front = false;
}

CarInterface::Steering SynchronizedCar::steer() const {
  return msgToSteer(mSteer.cmd);
}

void SynchronizedCar::steer(Steering cmd) {
  mSteer.cmd = steerToMsg(cmd);
}

void SynchronizedCar::windows(bool left, bool front, bool right){
  mWindows.left = left;
  mWindows.front = front;
  mWindows.right = right;
}
