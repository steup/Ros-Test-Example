#pragma once

#include "RoadInterface.h"

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

class OccupancyGridRoads : public::RoadInterface{
  private:
    nav_msgs::OccupancyGrid mGrid;
    ros::Publisher          mGridPub;
    ros::Timer              mTimer;

    void publishGrid(const ros::TimerEvent& e);

  public:
    OccupancyGridRoads(unsigned int maxY, unsigned int maxX, ros::Duration = ros::Duration(1));
    virtual Cell readCell(unsigned int y, unsigned int x) const;
    virtual void writeCell(unsigned int y, unsigned int x, Cell c);
    virtual bool isFree(unsigned int y, unsigned x) const { return readCell(y,x)==Cell::Free; }
    virtual bool isBlocked(unsigned int y, unsigned x) const { return readCell(y,x)==Cell::Blocked; }
    virtual bool isCar(unsigned int y, unsigned x) const { return readCell(y,x)==Cell::Car; }
};
