ament_add_gtest(costmap_test costmap_test.cpp)
target_link_libraries(costmap_test 
    explore_ros2_core
)

ament_add_gtest(frontier_search_test frontier_search_test.cpp)
target_link_libraries(frontier_search_test 
    explore_ros2_core
)
