#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace explore_ros2
{

    class CostmapClient
    {
    public:
        CostmapClient() {

        }

        void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
            
        }

        nav2_costmap_2d::Costmap2D *getCostmap() const
        {
            return costmap_;
        }

    private:
        nav2_costmap_2d::Costmap2D *costmap_;
    };

} // namespace explore_ros2
