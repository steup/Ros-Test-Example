#pragma once

#include "cars/Steer.h"
#include "cars/Windows.h"

#include <string>

class SynchronizedCar {
  protected:
    cars::Steer mSteer;
    cars::Windows mWindows;
    const std::string mPlate;
  public:
    enum class Steering {
      Straight,
      Left,
      Right
    };

  private:
    static Steering msgToSteer(int32_t cmd);
    static int32_t steerToMsg(Steering cmd);

  public:

    SynchronizedCar(const std::string numberPlate);
    const std::string& numberPlate() const { return mPlate; }
    virtual ~SynchronizedCar() {}
    virtual void steer(Steering cmd);
    Steering steer() const;
    virtual void windows(bool left, bool front, bool right);
    bool leftWindow()  const { return mWindows.left; }
    bool frontWindow() const { return mWindows.front; }
    bool rightWindow() const { return mWindows.right; }
};
