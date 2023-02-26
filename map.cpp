#include "map.h"

Map::Map()
    : _frame_grid(0, 0, CV_8UC1, 0),
      _grid(),
      _rows(0),
      _cols(0),
      _origin(Eigen::Vector3f::Identity()) {}

void Map::loadOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg_) {
  _rows = msg_->info.height;
  _cols = msg_->info.width;
  _resolution = msg_->info.resolution;
  _inverse_resolution = 1.0 / _resolution;
  _grid = msg_->data;
  _origin << msg_->info.origin.position.x, msg_->info.origin.position.y, 0.0;
  _frame_grid = cv::Mat(_rows, _cols, CV_8UC1, _grid.data());   
  
  //_grid.data() return a pointer to the first elemn of the grid array

  std::cerr << "-- oc -> data length: " << (msg_->data).size() << std::endl;
  std::cerr << "-- oc -> frame_grid size: " << sizeof(_frame_grid) << std::endl;
  std::cerr << "-- oc -> elem: " << (int)_grid.data()[0] << std::endl;
  std::cerr << "-- oc -> rows: " << _rows << std::endl;
  std::cerr << "-- oc -> cols: " << _cols  << std::endl;
  std::cerr << "_____occupancy grid loaded_____\n";
}