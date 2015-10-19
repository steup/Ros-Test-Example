#pragma once

#include "RoadInterface.h"
#include "SimCar.h"

#include <map>

class Simulation {
  private:
    RoadInterface& mRoads;
    using Storage = std::map<std::string, SimCar>;
    Storage mCars;

  public:
    Simulation(RoadInterface& roads);
    bool createCar(const std::string& numberPlate);
    bool removeCar(const std::string& numberPlate);
    void run();
};
