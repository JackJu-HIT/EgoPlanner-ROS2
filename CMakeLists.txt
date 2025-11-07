cmake_minimum_required(VERSION 3.8)
project(ego_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter  
    -Wno-unused-but-set-variable 
    -Wno-vla-cxx-extension  
    -Wno-deprecated-declarations 
  )
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost 1.89 REQUIRED)

set(EIGEN3_INCLUDE_DIR "/opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3/Eigen") 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

find_package(Boost 1.89 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(./planner/bspline_opt/include)
include_directories(./planner/path_searching/include)
include_directories(./planner/plan_env/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)
include_directories(./planner/include)

add_subdirectory(planner/plan_manage)
add_subdirectory(planner/bspline_opt)
add_subdirectory(planner/path_searching)
add_subdirectory(planner/plan_env)

# 编译节点
add_executable(motion_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
)

target_link_libraries(
  motion_plan
  bspline_opt  
  path_searching  
  local_map 
  ego_planner  
  ${Boost_LIBRARIES} 
)

# 安装节点
install(TARGETS
  motion_plan
  DESTINATION lib/${PROJECT_NAME})

ament_package()