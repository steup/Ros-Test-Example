#include "Roads.h"

using namespace std;

Roads::Roads() {
  for(unsigned int x = 0; x < maxX; x++)
    for(unsigned int y = 0; y < maxY; y++) {
      if ((x - 1) % 7 == 0 || (x - 1) % 7 == 1 || (y - 1) % 7 == 0 || (y - 1) % 7 == 1)
        mCells[y][x] = Cell::Free;
      else
        mCells[y][x] = Cell::Blocked;
      if(x == maxX -1 || y == maxY -1 || x == 0 || y == 0)
        mCells[y][x] = Cell::Blocked;
    }
  mGrid.info.resolution=5;
  mGrid.info.width=maxX;
  mGrid.info.height=maxY;
  mGrid.data.resize(maxX*maxY);
  mGridPub = mNode.advertise<nav_msgs::OccupancyGrid>("roads", 1000);
  mCarRegistry = mNode.advertiseService("carRegistry", &Roads::createCar, this);
}

bool Roads::createCar(cars::NewCar::Request& req, cars::NewCar::Response& res) {
  if ( mCars.find(req.numberPlate) != mCars.end()) {
      ROS_ERROR_STREAM("Tried to spawn second car with number plate: " << req.numberPlate);
      res.success = false;
      return true;
    }
  
  if( mCells[1][1] != Cell::Free) {
    ROS_ERROR_STREAM("Car spawn place already occupied!");
    res.success = false;
    return true; 
  }

  mCells[1][1] = Cell::Car;
  mCars.emplace(req.numberPlate, req.numberPlate);
  mCars.find(req.numberPlate)->second.windows(mCells[2][1]==Cell::Free, mCells[1][2]==Cell::Free, mCells[0][1]==Cell::Free);
  res.success = true;
    
  return true; 
}

void Roads::run() {
}
