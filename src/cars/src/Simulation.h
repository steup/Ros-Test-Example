#pragma once

#include "RoadInterface.h"
#include "SimCarInterface.h"
#include "SimCarFactoryInterface.h"

#include <map>

class Simulation {
  private:
    RoadInterface& mRoads;
    SimCarFactoryInterface& mFactory;
    using Storage = std::map<std::string, std::unique_ptr<SimCarInterface>>;
    Storage mCars;

  public:
    Simulation(RoadInterface& roads, SimCarFactoryInterface& factory);
    bool createCar(const std::string& numberPlate);
    bool removeCar(const std::string& numberPlate);
    void run();
};
