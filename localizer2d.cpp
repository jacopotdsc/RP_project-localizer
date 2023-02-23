#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"
#include "ros_bridge.h"
#include <typeinfo>

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  bool setMap_debug = true;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
 
  //std::cerr << _map->rows() << std::endl;
  //std::cerr << _map->initialized() << std::endl;
  //std::cerr << ( _map->rows() != 0 )<< std::endl;

  
  if(_map->initialized() == 0){
    std::cerr << "-- map not initialized" << std::endl;
    return;
  }

  Map world_map = (*_map);
  cv::Size2i matrix_size = _map->size();
  auto mat_rows = matrix_size.height;
  auto mat_cols = matrix_size.width;

  if( setMap_debug ){ std::cerr << "-- setMap -> rows: " << mat_rows << std::endl;
                      std::cerr << "-- setMap -> cols: " << mat_cols << std::endl; }

  /*
  auto it_start = world_map.begin();
  auto it_end = world_map.end();

  for(auto it = world_map.begin(); it != world_map.end(); it++){

    auto p = *it;
    auto point_in_grid = cv::Point2i(p);

    if(point_in_grid == CellType::Occupied){
        auto converted_point = _map->grid2world(point_in_grid);
        _obst_vect.push_back(converted_point);
        
        //std::cerr << converted_point << std::endl;
      }
  }*/

  for( int r=0; r < mat_rows; r++){
    for( int c=0; c < mat_cols; c++){

      //auto point_in_grid = world_map.at<cv::Point2i>(r,c);
      //std::cerr << point_in_grid << std::endl;

      auto point_in_grid = (*_map)(r,c);
      //auto converted_point = (*_map).grid2world(point_in_grid);

      //std::cerr << "qui----> " << point_in_grid << std::endl;

      if(point_in_grid == CellType::Occupied){
        float r_new = r * 0.05; // + (int)_map->origin().x();
        float c_new = c * 0.05; // + (int)_map->origin().y();

        Eigen::Vector2f converted_point = Eigen::Vector2f(r_new,c_new);
        _obst_vect.push_back(converted_point);
        
        //std::cerr << converted_point << std::endl;
      }
    }

  }

  // Create KD-Tree
  TreeType my_kd_tree(_obst_vect.begin(), _obst_vect.end(), 10);

 // _obst_tree_ptr.reset( &my_kd_tree);
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end(), 10);

  if(setMap_debug){ std::cerr << "-- setMap and kd-tree-> size: " << _obst_vect.size() << std::endl; }


}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {

  if(X().isApprox(Eigen::Isometry2f::Identity())){
    _laser_in_world.translation() = initial_pose_.translation()*0.05;
    _laser_in_world.linear() = initial_pose_.linear();
    std::cerr << "-- first pose " << std::endl;
  }
  else{
    _laser_in_world = initial_pose_;
  }
  std::cerr << "-- setted new pose" << std::endl;
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO

  ContainerType prediction;

  getPrediction(prediction);
  std::cerr << "-- predicted data length: " << prediction.size() << std::endl;

  /*
  if( scan_ != prediction){
    std::cerr << "-- different " << std::endl;
  }
  else{
    std::cerr << "-- equal " << std::endl;
  }*/

  // for debug
  //sensor_msgs::LaserScan::ConstPtr my_scan_msg = nullptr;
  //auto debug_scan_msg = eigen2scan(prediction, my_scan_msg, _range_min, _range_max, _angle_min, _angle_max, _angle_increment);
  //pub_scan.publish(my_scan_msg);
  /////////

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO

  ICP my_icp = ICP(scan_, prediction, 5);
  my_icp.X() = X();

  //std::cerr << "-- old: translation + rotation " << std::endl;
  //std::cerr << X().translation() << std::endl;
  //std::cerr << X().linear() << std::endl;
  my_icp.run(30);

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO

  Eigen::Isometry2f new_iso;
  new_iso.translation() = my_icp.X().translation();
  new_iso.linear() = my_icp.X().linear();

  setInitialPose(new_iso);
  std::cerr << "-- new: translation: [ " << X().translation()[0] << ", " << X().translation()[1] << " ]" << std::endl;
  //std::cerr << X().translation() << std::endl;
  //std::cerr << X().linear() << std::endl;
  std::cerr << "--------------------- " << std::endl;
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO

  TreeType::AnswerType neighbors;

  _obst_tree_ptr->fullSearch(neighbors, X().translation(), _range_max);
  //std::cerr << "-- neighbors: " << neighbors.size() << std::endl;

  for( auto n: neighbors){
    prediction_.push_back(*n);
  }
  return;

  /*
  for( auto ob : _obst_vect){
    
    auto dist_vector = X().translation() - ob;
    auto dist = dist_vector.norm();

    std::cerr << "-- dist_vector: \n" << dist_vector << std::endl;
    std::cerr << "-- norm: " << dist << std::endl;
    std::cerr << "-- obstacle: \n" << ob << std::endl;


    auto my_point = _obst_tree_ptr->bestMatchFast(ob, 10);
    neighbors.push_back(my_point);

    //std::cerr << "-- searched\n" << my_point << std::endl;

    /*
    if( dist <= _range_max){
      std::cerr << "-- searching\n";
     // _obst_tree_ptr->fastSearch(neighbors, X().translation(), 10);
    }*/
  

}