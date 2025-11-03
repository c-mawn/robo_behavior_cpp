#define _USE_MATH_DEFINES

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "neato2_interfaces/msg/bump.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class path_follow : public rclcpp::Node{
  public:
    path_follow() : Node("path_follow"){
      //create timer and publisher for controlling neato
      vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&path_follow::run_loop, this));
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&path_follow::odom_callback, this, std::placeholders::_1));
    }
  
  private:

    //create a point struct to hold xy pairs
    struct Point
    {
      double x;
      double y;
    };

    //hard coded path... TODO: make dynamically changable later
    std::vector<Point> path={
      {0.8, 0.2},
      {0.4, 0.4},
      {0.6, 0.3},
      {0.0, 0.0}
    };

    //current index of waypoint neato is at
    size_t current_waypoint = 0;

    //vars to store the current pose of neato 
    double current_x = 0.0;
    double current_y = 0.0;
    double current_theta = 0.0; //in rad

    /*
    Odometry subscriber callback function that takes in the current odom of the neato
    and updates the values in the class
    */
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){
      //sets the positions to current odom positions
      current_x = msg->pose.pose.position.x;
      current_y = msg->pose.pose.position.y;

      /*
      Find the current theta from the odom; odom message gives quaternion angles
      but we want euler angles. Conversion below is created by AI
      */
      double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
        msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
      double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
      
      //sets the current angle using the conversion above
      current_theta = std::atan2(siny_cosp, cosy_cosp);
    }
    

    void run_loop(){
      // creates velocity message to be populated
      auto vel = geometry_msgs::msg::Twist();

      RCLCPP_INFO_STREAM(this->get_logger(), "Current Pose: (x: " << current_x
                                              << ", y: " << current_y 
                                              << ", theta: " << current_theta << ")");
      
      RCLCPP_INFO_STREAM(this->get_logger(), "Target Waypoint #" << current_waypoint
                                              << ": (x: " << path[current_waypoint].x
                                              << ", y: " << path[current_waypoint].y << ")");

      /*
      Logic for calculating the velocity commands needs to get to the next waypoint
      */


      // publishes the velocity message
      vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_follow>());
  rclcpp::shutdown();
  return 0;
}
