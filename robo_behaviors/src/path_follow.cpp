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
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class path_follow : public rclcpp::Node{
  public:
    path_follow() : Node("path_follow"){
      //create timer, publishers, and subscriber for controlling neato
      vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/viz", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&path_follow::run_loop, this));
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&path_follow::odom_callback, this, std::placeholders::_1));

      //create ros params for tuning gain
      this->declare_parameter("lin_gain", 0.1);
      this->declare_parameter("ang_gain", 0.5);
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

    //controller gains, defaulted to 0.1, 0.5; ros params changable in terminal
    double linear_gain = this->get_parameter("lin_gain").as_double();
    double angular_gain = this->get_parameter("ang_gain").as_double();

    //distance threshold to decide whether we are close enough to the waypoint to continue
    double distance_threshold = 0.1;

    //max velocities (defined by neato capabilities)
    double max_linear_vel = 0.2;
    double max_angular_vel = 0.2;


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
    
    /*
    Run loop called by timer. calculates necessary velocities to traverse path
    and publishes to /cmd_vel topic
    */
    void run_loop(){
      // creates velocity message to be populated
      auto vel = geometry_msgs::msg::Twist();

      //update lin and ang gain from parameters
      linear_gain = this->get_parameter("lin_gain").as_double();
      angular_gain = this->get_parameter("ang_gain").as_double();

      //check if path is completed (went to all waypoints)
      if(current_waypoint >= path.size()){
        RCLCPP_INFO(this->get_logger(), "Path complete");
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub_->publish(vel);
        return;
      }

      //get target waypoint
      Point target_point = path[current_waypoint];

      //calculate the xy error
      double dx = target_point.x - current_x;
      double dy = target_point.y - current_y;

      //calculate the distance error
      double distance_error = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

      //calc angle to target
      double angle_to_target = std::atan2(dy, dx);

      //calc angle difference, then normalize to btwn +pi and -pi
      double angle_error = angle_to_target - current_theta;
      angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

      //control logic
      if(distance_error < distance_threshold){
        //this case means we are at the waypoint
        //logs the waypoint
        RCLCPP_INFO(this->get_logger(), "Reached a waypoint");
        //updates current waypoint
        current_waypoint++;

        //stops the robot for 1 cycle
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
      }
      else{
        //this case means we are not at the waypoint yet
        //calc velocities
        double linear_vel = linear_gain * distance_error;
        double angular_vel = angular_gain * angle_error;

        //limit velocities over the max
        linear_vel = std::min(linear_vel, max_linear_vel);
        angular_vel = std::max(angular_vel, -max_angular_vel);
        angular_vel = std::min(angular_vel, max_angular_vel);

        //set velocities
        vel.linear.x = linear_vel;
        vel.angular.z = angular_vel;

      }
      RCLCPP_INFO_STREAM(this->get_logger(), "linear vel: " << vel.linear.x << " | angular vel: " << vel.angular.z);
      //marker debug, shows the current goal point in rviz
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "odom";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = path[current_waypoint].x;
      marker.pose.position.y = path[current_waypoint].y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      viz_pub_->publish(marker);
      // publishes the velocity message
      vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_follow>());
  rclcpp::shutdown();
  return 0;
}
