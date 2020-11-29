#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace explore_ros2
{

    class Costmap
    {
    public:
        void updateFullMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void updatePartialMap(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

        const nav2_costmap_2d::Costmap2D *getCostmap() const;

        nav2_costmap_2d::Costmap2D *getCostmap();   

    private:
        nav2_costmap_2d::Costmap2D costmap_;

        std::string global_frame_;
    };

} // namespace explore_ros2
