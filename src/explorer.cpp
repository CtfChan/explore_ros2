#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "explore_ros2/explore.hpp"

using std::placeholders::_1;

namespace explore_ros2
{

  Explore::Explore() : Node("explore")
  {
    timer_ = this->create_wall_timer(
        std::chrono::seconds(planner_period_),
        std::bind(&Explore::makePlan, this));

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "topic", 10, std::bind(&Explore::mapReceived, this, _1));
  }

  void Explore::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got new map");

    // costmap_client_.update(msg)
  }
  void Explore::makePlan()
  {
    RCLCPP_INFO(this->get_logger(), "Making Plan");

    // frontiers = search_.searchFrom()
    // get best one
  }

} // namespace explore_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore_ros2::Explore>());
  rclcpp::shutdown();
  return 0;
}