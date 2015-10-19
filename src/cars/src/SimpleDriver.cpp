#include "SimpleDriver.h"

using namespace std;

void SimpleDriver::onWindowUpdate() {
  if(!frontWindow()) {
    if(leftWindow())
      steer(Steering::Left);
    else
      steer(Steering::Right);
  } else
    steer(Steering::Straight);
}
