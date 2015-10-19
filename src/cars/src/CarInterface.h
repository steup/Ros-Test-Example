#pragma once

#include "string"

class CarInterface {
  private:
    const std::string mNumberPlate;
  protected:
    virtual void onWindowUpdate() {}
  public:
    enum class Steering {
      Straight,
      Left,
      Right
    };

    CarInterface(const std::string numberPlate) : mNumberPlate(numberPlate) {}
    virtual ~CarInterface() {}
    const std::string& numberPlate() const { return mNumberPlate; }
    Steering steer() const;
    bool leftWindow()  const;
    bool frontWindow() const;
    bool rightWindow() const;
};
