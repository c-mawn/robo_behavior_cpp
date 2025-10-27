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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "neato2_interfaces/msg/bump.hpp"

using namespace std::chrono_literals;

class path_follow : public rclcpp::Node{
  public:
    path_follow() : Node("path_follow"){
      //create timer and publisher for controlling neato
      vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&path_follow::run_loop, this));
    }
  
  private:
    //define variables used throughout class

    //assuming that path for neato to follow is given in this variable
    //(can be changed in the future to be dynamically changed somehow)
    std::vector<std::pair<float, float>> calc_vel(
      const std::vector<std::vector<float>>& path, float dt){
      /*
      Function that takes in list of points for robot to traverse and returns
      a vector containing pairs of linear and angular velocities to command the robot
      */
     
    }

    void run_loop(){
      auto vel = geometry_msgs::msg::Twist();

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_follow>());
  rclcpp::shutdown();
  return 0;
}
