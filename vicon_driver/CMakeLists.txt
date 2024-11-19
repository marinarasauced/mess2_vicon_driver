cmake_minimum_required(VERSION 3.5)
project(vicon_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)

add_executable(driver src/vicon_driver/communicator.cpp src/vicon_driver/publisher.cpp)

# Link against yaml-cpp and other libraries
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(driver ViconDataStreamSDK_CPP ${Boost_LIBRARIES} yaml-cpp)

target_include_directories(
  driver PRIVATE
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

ament_target_dependencies(driver "rclcpp" "geometry_msgs")

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

install(TARGETS driver DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()