#pragma once

#include "CarInterface.h"
#include "cars/Steer.h"
#include "cars/Windows.h"

class SynchronizedCar : public CarInterface{
  protected:
    cars::Steer mSteer;
    cars::Windows mWindows;

  private:
    static Steering msgToSteer(int32_t cmd);
    static int32_t steerToMsg(Steering cmd);
  public:
    SynchronizedCar(const std::string numberPlate);
    virtual ~SynchronizedCar() {}
    virtual void steer(Steering cmd);
    virtual Steering steer() const;
    virtual void windows(bool left, bool front, bool right);
    virtual bool leftWindow()  const { return mWindows.left; }
    virtual bool frontWindow() const { return mWindows.front; }
    virtual bool rightWindow() const { return mWindows.right; }
};
