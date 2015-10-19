#include "OccupancyGridRoads.h"

using namespace std;

OccupancyGridRoads::OccupancyGridRoads( unsigned int maxY, unsigned int maxX, ros::Duration frameRate)
  : RoadInterface(maxY, maxX) {
  mGrid.info.resolution=5;
  mGrid.info.width=maxX;
  mGrid.info.height=maxY;
  mGrid.data.resize(maxX*maxY);
  for(unsigned int x = 0; x < maxX; x++)
    for(unsigned int y = 0; y < maxY; y++) {
      if ((x - 1) % 7 == 0 || (x - 1) % 7 == 1 || (y - 1) % 7 == 0 || (y - 1) % 7 == 1)
        writeCell(y,x, Cell::Free);
      else
        writeCell(y, x, Cell::Blocked);
      if(x == maxX -1 || y == maxY -1 || x == 0 || y == 0)
        writeCell(y, x, Cell::Blocked);
    }
  mGridPub = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>("roads", 1000);
  mTimer   = ros::NodeHandle().createTimer(frameRate, &OccupancyGridRoads::publishGrid, this);
  
}

OccupancyGridRoads::Cell OccupancyGridRoads::readCell(unsigned int y, unsigned int x) const {
  if(x>maxX() || y>maxY()) {
    ROS_ERROR_STREAM("Occupancy grid access out of bounds: (" << x << ", " << y << ") > (" << maxX() << ", " << maxY() << ")");
    return Cell::Blocked;
  }
  switch(mGrid.data[x+maxX()*y]) {
    case(0):   return Cell::Free;
    case(-1): return Cell::Car;
    case(100): return Cell::Blocked;
    default  : return Cell::Blocked;
  }
}

void OccupancyGridRoads::writeCell(unsigned int y, unsigned int x, Cell cell) {
  if(x>maxX() || y>maxY()) {
    ROS_ERROR_STREAM("Ouccpancy grid access out of bounds: (" << x << ", " << y << ") > (" << maxX() << ", " << maxY() << ")");
    return;
  }
  switch(cell) {
    case(Cell::Free)   : mGrid.data[x+maxX()*y] = 0;
                         break;
    case(Cell::Car)    : mGrid.data[x+maxX()*y] = -1;
                         break;
    case(Cell::Blocked): mGrid.data[x+maxX()*y] = 100;
                         break;
    default            : mGrid.data[x+maxX()*y] = 100;
  }
}

void OccupancyGridRoads::publishGrid(const ros::TimerEvent& e) {
  mGridPub.publish(mGrid);
}
