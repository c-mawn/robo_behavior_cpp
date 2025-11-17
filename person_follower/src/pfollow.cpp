#define _USE_MATH_DEFINES

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "neato2_interfaces/msg/bump.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;


class pfollow : public rclcpp::Node
{
  public: 
    pfollow() : Node("pfollow")
    {
      //code here
      vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&pfollow::run_loop, this));
      scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&pfollow::get_item_error, this, _1)
      );
    }
    
  private:
    //define variables used in computaiton
    double item_error = 0.0;
    double avg_x = 0.0;
    double avg_y = 0.0;
    double item_distance = 0.0;


    void run_loop(){
      //create new twist object
      auto vel = geometry_msgs::msg::Twist();

      //check if the item error is high, if it is, then set the linear speed to 0
      // if the item error isnt too high, then move forward and turn at the same time
      if(std::abs(item_error) > 0.5){
        vel.linear.x = 0.0;
      } else{
        vel.linear.x = 0.2;
      }
      //sets the angular speed to always be .75 times the item error, so that it
      //scales with the amount you need to turn
      //then publish
      vel.angular.z = 0.75 * item_error;
      vel_pub_->publish(vel);
    }

    void get_item_error(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      std::vector<float> scans = msg->ranges;
      std::vector<int> ranges(scans.size());
      std::iota(ranges.begin(), ranges.end(), 0);
      static const double d_to_rad = M_PI / 180.0;

      // remove scans that are too far away or <= 0
      for (size_t i = 0; i < scans.size(); ) {
        float current_scan = scans[i];
        if (current_scan > 1.0f || current_scan <= 0.0f) {
          // erase the scan and the corresponding index in ranges
          scans.erase(scans.begin() + i);
          ranges.erase(ranges.begin() + i);
          // do not increment i: next element shifted into i
        } else {
          ++i;
        }
      }

      if (scans.size() > 0) {
        //create vectors of x and y coords and reserve space in them to be filled
        std::vector<float> x_coordinates;
        std::vector<float> y_coordinates;
        x_coordinates.reserve(scans.size());
        y_coordinates.reserve(scans.size());

        //change the scans to cartesian coordinates
        for (size_t i = 0; i < scans.size(); ++i) {
            double angle_in_rad = static_cast<double>(ranges[i]) * d_to_rad;
            x_coordinates.push_back(scans[i] * std::cos(angle_in_rad));
            y_coordinates.push_back(scans[i] * std::sin(angle_in_rad));
        }

        //averages the coordinates to find the center of the item
        double sum_x = std::accumulate(x_coordinates.begin(), x_coordinates.end(), 0.0);
        double sum_y = std::accumulate(y_coordinates.begin(), y_coordinates.end(), 0.0);

        avg_x = sum_x / x_coordinates.size();
        avg_y = sum_y / y_coordinates.size();
 
        // math to find the distance and angle
        item_distance = std::sqrt(std::pow(avg_x, 2) + std::pow(avg_y, 2));
        double theta = std::atan2(avg_y, avg_x);

        item_error = theta;
      } 
      else {
        item_error = 0.0;
        avg_x = 0.0;
        avg_y = 0.0;
        item_distance = 0.0;
      }
    }
    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    size_t count_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pfollow>());
  rclcpp::shutdown();
  return 0;
}
