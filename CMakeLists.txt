cmake_minimum_required(VERSION 3.8)
project(nturt_led_controller)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
        include
)

# libraries
# library: nturt_led_controller
add_library(${PROJECT_NAME} SHARED
        src/led_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
        wiringPi
)
ament_target_dependencies(${PROJECT_NAME}
        can_msgs
        rcl_interfaces
        rclcpp
        rclcpp_components
)
# register to node components
# node component: LedController
rclcpp_components_register_nodes(nturt_led_controller "LedController")
set(node_plugins "${node_plugins}LedController;$<TARGET_FILE:nturt_led_controller>\n")

# applications
# executable: nturt_led_controller_node
add_executable(${PROJECT_NAME}_node
        app/led_controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# install libraries
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

# install applications, test executables
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
endif()

# export package dependencies
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)
ament_export_dependencies(
        ament_cmake
        can_msgs
        rcl_interfaces
        rclcpp
)

ament_package()
