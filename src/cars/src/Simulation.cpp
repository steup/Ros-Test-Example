#include "Simulation.h"

#include <ros/ros.h>

#include <tuple>
#include <list>

using namespace std;
using Steering = SynchronizedCar::Steering;
using Cell     = RoadInterface::Cell;
using Dir      = SimCarInterface::Dir;

Simulation::Simulation(RoadInterface& roads, SimCarFactoryInterface& factory) : mRoads(roads), mFactory(factory){}

void Simulation::run() {

  list<string> eraseList;

  for(auto& v : mCars) {
    SimCarInterface& car = *v.second;

    if(car.steer() == Steering::Left)
      switch(car.dir) {
        case(Dir::Up)   : car.dir = Dir::Left;
                          break;
        case(Dir::Left) : car.dir = Dir::Down;
                          break;
        case(Dir::Down) : car.dir = Dir::Right;
                          break;
        case(Dir::Right): car.dir = Dir::Up;
                          break;
      }
    
    if(car.steer() == Steering::Right)
      switch(car.dir) {
        case(Dir::Up)   : car.dir = Dir::Right;
                          break;
        case(Dir::Left) : car.dir = Dir::Up;
                          break;
        case(Dir::Down) : car.dir = Dir::Left;
                          break;
        case(Dir::Right): car.dir = Dir::Down;
                          break;
      }
      
    mRoads.writeCell(car.y, car.x, Cell::Free);
    unsigned int oldX = car.x, oldY=car.y;
    switch(car.dir) {
      case(Dir::Up)   : car.y+=1;
                        break;
      case(Dir::Left) : car.x-=1;
                        break;
      case(Dir::Down) : car.y-=1;
                        break;
      case(Dir::Right): car.x+=1;
                        break;
    }


    if(!mRoads.isFree(car.y, car.x)) {
      eraseList.push_back(v.first);
      mRoads.writeCell(oldY, oldX, Cell::Blocked);
      continue;
    }
    mRoads.writeCell(car.y, car.x, Cell::Car);

  }

  for(auto& v : eraseList)
    mCars.erase(v);

  for(auto& v : mCars) {
    SimCarInterface& car = *v.second;

    switch(car.dir) {
      case(Dir::Up)   : car.windows(mRoads.isFree(car.y, car.x-1),
                                    mRoads.isFree(car.y+1, car.x),
                                    mRoads.isFree(car.y, car.x+1));
                        break;
      case(Dir::Left) : car.windows(mRoads.isFree(car.y-1, car.x),
                                    mRoads.isFree(car.y, car.x-1),
                                    mRoads.isFree(car.y+1, car.x));
                        break;
      case(Dir::Down) : car.windows(mRoads.isFree(car.y, car.x+1),
                                    mRoads.isFree(car.y-1, car.x),
                                    mRoads.isFree(car.y, car.x-1));
                        break;
      case(Dir::Right): car.windows(mRoads.isFree(car.y+1, car.x),
                                    mRoads.isFree(car.y, car.x+1),
                                    mRoads.isFree(car.y-1, car.x));
                        break;
    }
  }
}

bool Simulation::createCar(const std::string& numberPlate){
  if ( mCars.find(numberPlate) != mCars.end()) {
      ROS_ERROR_STREAM("Tried to spawn second car with number plate: " << numberPlate);
      return false;
    }
  if ( !mRoads.isFree(1,1) ) {
      ROS_ERROR_STREAM("Tried to spawn a car on a occupied space!");
      return false;
  }
  
  auto i = mCars.emplace(numberPlate, mFactory.createCar(numberPlate, Dir::Right, 1, 1));
  SimCarInterface& newCar = *i.first->second;
  newCar.windows(mRoads.isFree(newCar.y+1,newCar.x), mRoads.isFree(newCar.y,newCar.x+1), mRoads.isFree(newCar.y-1, newCar.x));
  return true; 
}

bool Simulation::removeCar(const std::string& numberPlate) {
  return mCars.erase(numberPlate);
}
