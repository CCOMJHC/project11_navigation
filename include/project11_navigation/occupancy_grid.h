#ifndef PROJECT11_NAVIGATION_OCCUPANCY_GRID_H
#define PROJECT11_NAVIGATION_OCCUPANCY_GRID_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace project11_navigation
{

class OccupancyGrid
{
public:
  OccupancyGrid(const nav_msgs::OccupancyGrid &grid);

  const nav_msgs::OccupancyGrid& message() const;
  const int8_t getValue(const geometry_msgs::Point &point) const;
private:
  nav_msgs::OccupancyGrid grid_;

};

} // namespace project11

#endif
