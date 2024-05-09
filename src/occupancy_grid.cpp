#include <project11_navigation/occupancy_grid.h>

namespace project11_navigation
{

OccupancyGrid::OccupancyGrid(const nav_msgs::OccupancyGrid& grid)
  :grid_(grid)
{

}

const nav_msgs::OccupancyGrid& OccupancyGrid::message() const
{
  return grid_;
}

const int8_t OccupancyGrid::getValue(const geometry_msgs::Point &point) const
{
  int i = (point.x - grid_.info.origin.position.x)/grid_.info.resolution;
  int j = (point.y - grid_.info.origin.position.y)/grid_.info.resolution;
  if(i >= 0 && i < grid_.info.width && j >= 0 && j < grid_.info.height)
  {
    return grid_.data[j*grid_.info.width+i];
  }

  return -1;
}

} // namespace project11_navigation
