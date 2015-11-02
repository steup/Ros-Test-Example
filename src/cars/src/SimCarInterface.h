#pragma once

#include "SynchronizedCar.h"

class SimCarInterface : public SynchronizedCar{
  public:
    enum class Dir {
      Up, Down, Left, Right
    };
  protected:
    Dir mDir;
    unsigned int mX,  mY;
  public:

    SimCarInterface(const std::string numberPlate, Dir dir, unsigned int x, unsigned int y) : SynchronizedCar(numberPlate), mDir(dir), mX(x), mY(y) {}
    virtual ~SimCarInterface() {}

    Dir dir() const { return mDir; }
    unsigned int x() const { return mX; }
    unsigned int y() const { return mY; }

};
