#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"


// Map callback definition
void callback_map(const nav_msgs::OccupancyGrid::ConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScan::ConstPtr&);


std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;
ros::Subscriber sub_map, sub_initialpose, sub_scan;

//tf2_ros::TransformBroadcaster br; 

Localizer2D localizer;


int main(int argc, char** argv) {
  // Initialize ROS system
  ros::init(argc, argv, "localizer_node");

  ROS_INFO("Node started.");

  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  
  localizer = Localizer2D();
  Map my_map = Map();
  map_ptr.reset(&my_map);

  /*
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   *
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  
  
  //ros::Subscriber sub_map =         nh.subscribe("/map/nav_msgs/OccupancyGrid", 1000, callback_map);
  //ros::Subscriber sub_initialpose = nh.subscribe("/initialpose/geometry_msgs/PoseWithCovarianceStamped", 1000, callback_initialpose);
  //ros::Subscriber sub_scan =        nh.subscribe("/base_scan/sensor_msgs/LaserScan", 1000, callback_scan);

  sub_map =         nh.subscribe("/map", 1000, callback_map);
  sub_initialpose = nh.subscribe("/initialpose", 1000, callback_initialpose);
  sub_scan =        nh.subscribe("/base_scan", 1000, callback_scan);

  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out",10);  

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);
  
  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGrid::ConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  ROS_INFO("I heard: OccupancyGrid_msg ");

  if(!map_ptr->initialized()){
    map_ptr->loadOccupancyGrid(msg_);


    //std::cerr << map_ptr->rows() << std::endl;
    //std::cerr << map_ptr->cols() << std::endl;
    //std::cerr << map_ptr->initialized() << std::endl;

    std::cerr << "-- map_ptr, occupancy grid" << map_ptr << std::endl;
    std::cerr << "-- loadOccupancyGrid\n";


    localizer.setMap(map_ptr);
    std::cerr << "-- setMap\n";
  }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_) {
    // const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_  
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */
   
  
    ROS_INFO("I heard: PoseWithCovarianceStamped_msg ");

    Eigen::Isometry2f iso;
    pose2isometry(msg_->pose.pose,iso);
    localizer.setInitialPose(iso);
    std::cerr << "-- setInitialPose\n";

}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */

  tf2_ros::TransformBroadcaster br;
  //ROS_INFO("I heard: LaserScan_msg ");

  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> my_vect;
  scan2eigen(msg_, my_vect);

  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  
  float rmin = msg_->range_min;
  float rmax = msg_->range_max;
  float amin = msg_->angle_min;
  float amax = msg_->angle_max;
  float ainc = msg_->angle_increment;

  localizer.setLaserParams( rmin, rmax, amin, amax, ainc );
  //std::cerr << "-- setLaserParams\n";


  /** 
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  // TODO

  /*
  void isometry2transformStamped(const Eigen::Isometry2f& pose_,
                               geometry_msgs::TransformStamped& msg_,
                               const std::string& frame_id_,
                               const std::string& child_frame_id_,
                               const ros::Time& stamp_) {
  */

  Eigen::Isometry2f pose_laser_world = localizer.X();

  // must do a transformation, find it

  geometry_msgs::TransformStamped transform_stamped_message;
  isometry2transformStamped(pose_laser_world, 
                            transform_stamped_message,
                            FRAME_WORLD,
                            FRAME_LASER,
                            msg_->header.stamp);

  br = tf2_ros::TransformBroadcaster();
  br.sendTransform(transform_stamped_message);

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */

  nav_msgs::Odometry odometry_message;
  transformStamped2odometry(transform_stamped_message,
                            odometry_message);

  pub_odom.publish(odometry_message);

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;

  out_scan .scan_time =out_scan .scan_time/1000000000.0;
  out_scan .time_increment = out_scan .time_increment/1000000000.0;

  pub_scan.publish(out_scan);
}