#pragma once

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace explore_ros2
{

    struct Frontier
    {
        std::uint32_t size;
        double min_distance;
        double cost;
        geometry_msgs::msg::Point initial;
        geometry_msgs::msg::Point centroid;
        geometry_msgs::msg::Point middle;
        std::vector<geometry_msgs::msg::Point> points;
    };

    class FrontierSearch
    {
    public:
        FrontierSearch();

        FrontierSearch(nav2_costmap_2d::Costmap2D *costmap, double potential_scale,
                       double gain_scale, double min_frontier_size);

        std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

    private:
        Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                  std::vector<bool> &frontier_flag);

        bool isNewFrontierCell(unsigned int idx,
                               const std::vector<bool> &frontier_flag);

        double frontierCost(const Frontier &frontier);


        // private variables
        nav2_costmap_2d::Costmap2D *costmap_;
        unsigned char *map_;
        unsigned int size_x_, size_y_;
        double potential_scale_, gain_scale_;
        double min_frontier_size_;
    };

} // namespace explore_ros2