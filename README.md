# EgoPlanner-ROS2
ego planner后端基于ROS2交互式仿真

# how to use ?

ROS2 compile

1.运行节点motion_plan
2.运行rviz2
3.设置好全局轨迹与障碍物信息
4.ros2 topic pub /trigger_plan std_msgs/Bool  "{data"true}" 去触发规划
   