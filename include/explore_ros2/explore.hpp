#pragma once

#include "rclcpp/rclcpp.hpp"

#include "explore_ros2/frontier_search.hpp"
#include "explore_ros2/costmap.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace explore_ros2
{

    class Explore : public rclcpp::Node
    {
    public:
        Explore();

    private:
        void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void makePlan();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;

        unsigned int planner_period_ = 10; // in secs

        Costmap costmap_client_;
        FrontierSearch search_;
    };

} // namespace explore_ros2