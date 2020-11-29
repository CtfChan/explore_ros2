#include "nav_msgs/msg/occupancy_grid.hpp"
#include "explore_ros2/costmap.hpp"

#include <gtest/gtest.h>


using namespace explore_ros2;

TEST(CostmapTest, UpdateFullMapTest)
{
    // create msg
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    msg->info.height = 10;
    msg->info.width = 100;
    msg->info.resolution = 5;
    msg->info.origin.position.x = 2.0;
    msg->info.origin.position.y = 3.0;

    // place into costmap
    Costmap costmap;
    costmap.updateFullMap(msg);

    // verify values
    auto raw_map = costmap.getCostmap();
    EXPECT_EQ(raw_map->getSizeInCellsX(), msg->info.width);
    EXPECT_EQ(raw_map->getSizeInCellsY(), msg->info.height);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}