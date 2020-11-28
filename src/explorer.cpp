#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "explore_ros2/explore.hpp"

using std::placeholders::_1;

namespace explore_ros2
{
  Explore::Explore() : Node("explore")
  {
    // read params
    this->get_parameter_or<double>("potential_scale", potential_scale_, 1e-3);
    this->get_parameter_or<double>("gain_scale", gain_scale_, 1.0);
    this->get_parameter_or<double>("min_frontier_size", min_frontier_size_, 0.5);
    this->get_parameter_or<double>("orientation_scale", orientation_scale_, 0.0);

    this->get_parameter_or<int>("progress_timeout", timeout_, 30);
    progress_timeout_ = rclcpp::Duration(timeout_, 0.0);

    this->get_parameter_or<bool>("visualize", visualize_, true);

    // init transform server
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // init frontier_exploration
    search_ = FrontierSearch(costmap_.getCostmap(),
                             potential_scale_, gain_scale_,
                             min_frontier_size_);

    // timed callback to make plan
    unsigned int planner_period = 10; // in secs
    timer_ = this->create_wall_timer(
        std::chrono::seconds(planner_period),
        std::bind(&Explore::makePlan, this));

    // map subscribers
    // ros2 has no waitForMessage yet
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&Explore::mapReceived, this, _1));

    costmap_updates_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        "map_updates", 10, std::bind(&Explore::mapUpdateReceived, this, _1));

    // visualization publisher
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 1);

    // action client
    nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "navigate_to_pose");
  }

  void Explore::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got new map");
    costmap_.updateFullMap(msg);
  }

  void Explore::mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got new partial map");
    costmap_.updatePartialMap(msg);
  }

  void Explore::makePlan()
  {
    RCLCPP_INFO(this->get_logger(), "Making Plan");

    // get current pose
    // geometry_msgs::msg::Pose curr_pose = getRobotPose();
    geometry_msgs::msg::TransformStamped map_to_base_link;
    try
    {
      map_to_base_link = tf_buffer_->lookupTransform(
          "map", "base_link", tf2::TimePointZero);
      RCLCPP_INFO(this->get_logger(), "Got pose");
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_INFO(this->get_logger(), "No pose");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = map_to_base_link.transform.translation.x;
    pose.position.y = map_to_base_link.transform.translation.y;
    pose.position.z = map_to_base_link.transform.translation.z;

    auto frontiers = search_.searchFrom(pose.position);

    // stop system when frontiers are empty
    if (frontiers.empty())
    {
      // stop();
      return;
    }

    // publish
    if (visualize_)
    {
      visualizeFrontiers(frontiers);
    }

    // try to find non blacklisted position to go to
    auto frontier = std::find_if_not(frontiers.begin(), frontiers.end(),
                                     [this](const Frontier &f) {
                                       return goalOnBlacklist(f.centroid);
                                     });
    if (frontier == frontiers.end())
    {
      // stop();
      return;
    }
    geometry_msgs::msg::Point target_position = frontier->centroid;

    // time out if we are not making any progress
    bool same_goal = prev_goal_ == target_position;
    prev_goal_ = target_position;
    if (!same_goal || prev_distance_ > frontier->min_distance)
    {
      // we have different goal or we made some progress
      last_progress_ = this->now();
      prev_distance_ = frontier->min_distance;
    }
    // black list if we've made no progress for a long time
    if (this->now() - last_progress_ > progress_timeout_)
    {
      frontier_blacklist_.push_back(target_position);
      RCLCPP_DEBUG(this->get_logger(), "Adding current goal to black list");
      makePlan();
      return;
    }

    // we don't need to do anything if we still pursuing the same goal
    if (same_goal)
    {
      return;
    }

    // send goal to nav2
    ClientT::Goal client_goal;
    client_goal.pose.pose.position = target_position;
    client_goal.pose.pose.orientation.w = 1.;
    client_goal.pose.header.frame_id = "map";
    client_goal.pose.header.stamp = this->now();

    auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&Explore::goalResponseCallback, this, std::placeholders::_1);
    future_goal_handle_ =
        nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
  }

  void Explore::goalResponseCallback(
      std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
  {
    RCLCPP_INFO(this->get_logger(), "Reached Goal");
  }   

  bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point &goal)
  {

    constexpr static size_t tolerace = 5;
    nav2_costmap_2d::Costmap2D *costmap2d = costmap_.getCostmap();

    // check if a goal is on the blacklist for goals that we're pursuing
    for (auto &frontier_goal : frontier_blacklist_)
    {
      double x_diff = fabs(goal.x - frontier_goal.x);
      double y_diff = fabs(goal.y - frontier_goal.y);

      if (x_diff < tolerace * costmap2d->getResolution() &&
          y_diff < tolerace * costmap2d->getResolution())
        return true;
    }
    return false;
  }

  void Explore::visualizeFrontiers(const std::vector<Frontier> &frontiers)
  {
    static size_t prev_marker_count = 0;

    visualization_msgs::msg::MarkerArray markers_msg;

    // recycle m object when adding to marker array
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "frontier";
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(
        0, 1); // uniform distribution between 0 and 1

    size_t id = 0;
    // add centroid of wavefronts to marker array
    for (auto &wavefront : frontiers)
    {
      m.color.r = dis(gen);
      m.color.g = dis(gen);
      m.color.b = dis(gen);
      for (auto &pt : wavefront.points)
      {
        m.id = id++;
        m.pose.position.x = pt.x;
        m.pose.position.y = pt.y;
        m.pose.position.z = 0.0;
        markers_msg.markers.push_back(m);
      }
    }
    size_t curr_marker_count = markers_msg.markers.size();

    // delete prev. markers
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (; id < prev_marker_count; ++id)
    {
      m.id = int(id);
      markers_msg.markers.push_back(m);
    }

    prev_marker_count = curr_marker_count;
    marker_array_publisher_->publish(markers_msg);
  }

} // namespace explore_ros2

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explore_ros2::Explore>());
  rclcpp::shutdown();
  return 0;
}