#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "explore_ros2/frontier_search.hpp"
#include "explore_ros2/costmap.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/transform_listener.h"

#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace explore_ros2
{

    class Explore : public rclcpp::Node
    {
    public:
        using ClientT = nav2_msgs::action::NavigateToPose;
        using ActionClient = rclcpp_action::Client<ClientT>;

        Explore();

    private:
        void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

        void goalResponseCallback(
            std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future);

        void makePlan();

        void visualizeFrontiers(const std::vector<Frontier> &frontiers);

        bool goalOnBlacklist(const geometry_msgs::msg::Point &goal);

        geometry_msgs::msg::Pose getRobotPose() const;

        // publishers and subscribers
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr costmap_sub_;
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::ConstSharedPtr costmap_updates_sub_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

        // action client for nav2
        ActionClient::SharedPtr nav_to_pose_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;

        // tf
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // custom classes
        Costmap costmap_;
        FrontierSearch search_;

        // params
        double potential_scale_, orientation_scale_, gain_scale_, min_frontier_size_;
        bool visualize_;
        int timeout_;

        // places you should stop trying to explore
        std::vector<geometry_msgs::msg::Point> frontier_blacklist_;

        // remember prev goal
        geometry_msgs::msg::Point prev_goal_;
        double prev_distance_;
        rclcpp::Time last_progress_;
        rclcpp::Duration progress_timeout_{0, 0};
    };

} // namespace explore_ros2