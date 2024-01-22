#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    ros::Rate loop_rate(1);
    std::cout << "started";

    while (1) {
        std::cout << "No need to store this string\n";
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";

        pose_msg.pose.position.x = 1.0;
        pose_msg.pose.position.y = 2.0;
        pose_msg.pose.position.z = 3.0;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        pose_pub.publish(pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}