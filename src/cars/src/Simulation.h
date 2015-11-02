#pragma once

#include "RoadInterface.h"
#include "SimCarInterface.h"

#include <ros/ros.h>

#include <string>
#include <map>
#include <tuple>
#include <list>
#include <stdexcept>

template<typename SimCar>
class Simulation {
  private:
    RoadInterface& mRoads;
    using Storage = std::map<std::string, SimCar>;
    using Dir = SimCarInterface::Dir;
    using Steering = SimCarInterface::Steering;
    using Cell = RoadInterface::Cell;
    Storage mCars;

  public:
    Simulation(RoadInterface& roads) : mRoads(roads) {}

    const SimCarInterface& operator[](const std::string& numberPlate) const { 
      auto iter = mCars.find(numberPlate);
      if(iter == mCars.end())
        throw std::invalid_argument("No such car");
      return iter->second;
    }
    
    SimCarInterface& operator[](const std::string& numberPlate) { 
      auto iter = mCars.find(numberPlate);
      if(iter == mCars.end())
        throw std::invalid_argument("No such car");
      return iter->second;
    }

    bool createCar(const std::string& numberPlate) {
      if ( mCars.find(numberPlate) != mCars.end()) {
          ROS_ERROR_STREAM("Tried to spawn second car with number plate: " << numberPlate);
          return false;
      }
      if ( !mRoads.isFree(1,1) ) {
          ROS_ERROR_STREAM("Tried to spawn a car on a occupied space!");
          return false;
      }
      
      auto result = mCars.emplace( std::piecewise_construct,
                                   std::make_tuple(numberPlate), 
                                   std::make_tuple(numberPlate, Dir::Right, 1, 1)
                                  );
      SimCarInterface& newCar = result.first->second;
      newCar.windows( mRoads.isFree(newCar.y()+1,newCar.x()),
                      mRoads.isFree(newCar.y(),newCar.x()+1),
                      mRoads.isFree(newCar.y()-1, newCar.x())
                      );
      mRoads.writeCell(1, 1, Cell::Car);
      return true; 
    }

    void run() {
      std::list<std::string> eraseList;

      for(auto& v : mCars) {
        SimCar& car = v.second;

        if(car.steer() == Steering::Left)
          switch(car.dir()) {
            case(Dir::Up)   : car.dir(Dir::Left);
                              break;
            case(Dir::Left) : car.dir(Dir::Down);
                              break;
            case(Dir::Down) : car.dir(Dir::Right);
                              break;
            case(Dir::Right): car.dir(Dir::Up);
                              break;
          }
        
        if(car.steer() == Steering::Right)
          switch(car.dir()) {
            case(Dir::Up)   : car.dir(Dir::Right);
                              break;
            case(Dir::Left) : car.dir(Dir::Up);
                              break;
            case(Dir::Down) : car.dir(Dir::Left);
                              break;
            case(Dir::Right): car.dir(Dir::Down);
                              break;
          }
          
        mRoads.writeCell(car.y(), car.x(), Cell::Free);
        unsigned int oldX = car.x(), oldY=car.y();
        switch(car.dir()) {
          case(Dir::Up)   : car.y(car.y()+1);
                            break;
          case(Dir::Left) : car.x(car.x()-1);
                            break;
          case(Dir::Down) : car.y(car.y()-1);
                            break;
          case(Dir::Right): car.x(car.x()+1);
                            break;
        }


        if(!mRoads.isFree(car.y(), car.x())) {
          eraseList.push_back(v.first);
          mRoads.writeCell(oldY, oldX, Cell::Blocked);
          continue;
        }
        mRoads.writeCell(car.y(), car.x(), Cell::Car);

      }

      for(auto& v : eraseList)
        mCars.erase(v);

      for(auto& v : mCars) {
        SimCarInterface& car = v.second;

        switch(car.dir()) {
          case(Dir::Up)   : car.windows(mRoads.isFree(car.y(), car.x()-1),
                                        mRoads.isFree(car.y()+1, car.x()),
                                        mRoads.isFree(car.y(), car.x()+1));
                            break;
          case(Dir::Left) : car.windows(mRoads.isFree(car.y()-1, car.x()),
                                        mRoads.isFree(car.y(), car.x()-1),
                                        mRoads.isFree(car.y()+1, car.x()));
                            break;
          case(Dir::Down) : car.windows(mRoads.isFree(car.y(), car.x()+1),
                                        mRoads.isFree(car.y()-1, car.x()),
                                        mRoads.isFree(car.y(), car.x()-1));
                            break;
          case(Dir::Right): car.windows(mRoads.isFree(car.y()+1, car.x()),
                                        mRoads.isFree(car.y(), car.x()+1),
                                        mRoads.isFree(car.y()-1, car.x()));
                            break;
        }
      }
    }
};
