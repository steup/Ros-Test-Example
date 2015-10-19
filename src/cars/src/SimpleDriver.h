#include "Car.h"

class SimpleDriver : public Car {
  protected:
    virtual void onWindowUpdate();
  public:
    SimpleDriver(const std::string& numberPlate) : Car(numberPlate) {}
    virtual ~SimpleDriver() {}
};
