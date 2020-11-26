#include "nav_msgs/msg/occupancy_grid.hpp"
#include "explore_ros2/frontier_search.hpp"

#include <gtest/gtest.h>


using namespace explore_ros2;

TEST(FrontierSearchTest, EasyTest)
{
    ASSERT_EQ(0, 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}