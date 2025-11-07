/*
 * @Function: Ego Planner + RViz Interactive Input (Global Path + Obstacles)
 * @Create by: juchunyu@qq.com
 * @Date: 2025-11-01 18:58:01
 */
#include "trajectory_obstacles_publisher.h"

TrajectoryAndObstaclesPublisher::TrajectoryAndObstaclesPublisher() 
    : Node("ego_planner_interactive_node"),
      has_valid_global_path_(false),
      has_obstacles_(false),
      should_plan_(false),
      needs_replan_(false)
{
    // 1. 创建发布者（可视化用）
    global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_global_path", 10);
    local_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("visual_local_trajectory", 10);
    obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visual_obstacles", 10);

    // 2. 创建订阅者（接收RViz下发的数据）
    // 全局路径输入方式1：直接发布Path消息
    // rviz_global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    //     "/rviz_input_global_path",
    //     10,
    //     std::bind(&TrajectoryAndObstaclesPublisher::rviz_global_path_callback, this, std::placeholders::_1)
    // );

    // 全局路径输入方式2：使用2D Nav Goal工具设置终点
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::goal_pose_callback, this, std::placeholders::_1)
    );

    // // 障碍物输入方式1：发布PointCloud2消息
    // rviz_obstacles_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/rviz_input_obstacles",
    //     10,
    //     std::bind(&TrajectoryAndObstaclesPublisher::rviz_obstacles_callback, this, std::placeholders::_1)
    // );

    // // 障碍物输入方式2：使用Publish Point工具逐个添加
    // rviz_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    //     "/clicked_point",
    //     10,
    //     std::bind(&TrajectoryAndObstaclesPublisher::rviz_point_callback, this, std::placeholders::_1)
    // );

    // 障碍物输入方式3：使用2D Pose Estimate工具添加
    pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::pose_estimate_callback, this, std::placeholders::_1)
    );

    // 3. 触发规划的话题
    trigger_plan_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_plan",
        10,
        std::bind(&TrajectoryAndObstaclesPublisher::trigger_plan_callback, this, std::placeholders::_1)
    );

    // 4. 初始化Ego Planner基础配置
    init_ego_planner_base();

    // 5. 定时器：5Hz触发规划与发布
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrajectoryAndObstaclesPublisher::publish_and_plan, this)
    );

    RCLCPP_INFO(this->get_logger(), "Interactive Ego Planner Node Ready!");
    RCLCPP_INFO(this->get_logger(), "=== 使用方法 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加全局路径：使用2D Nav Goal工具设置终点");
    RCLCPP_INFO(this->get_logger(), "2. 添加障碍物（三种方法任选）：");
    RCLCPP_INFO(this->get_logger(), "   - 方法A：使用Publish Point工具点击地图");
    RCLCPP_INFO(this->get_logger(), "   - 方法B：使用2D Pose Estimate工具点击Grid任意位置（推荐）");
    RCLCPP_INFO(this->get_logger(), "   - 方法C：发布PointCloud2到 /rviz_input_obstacles");
    RCLCPP_INFO(this->get_logger(), "3. 触发规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: true}\"");
    RCLCPP_INFO(this->get_logger(), "4. 停止规划：ros2 topic pub /trigger_plan std_msgs/Bool \"{data: false}\"");
    RCLCPP_INFO(this->get_logger(), "5. 支持多次规划：可以反复发送true/false控制规划");
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "=== RViz2设置步骤 ===");
    RCLCPP_INFO(this->get_logger(), "1. 添加Grid显示");
    RCLCPP_INFO(this->get_logger(), "2. 添加2D Nav Goal工具（话题：/goal_pose）");
    RCLCPP_INFO(this->get_logger(), "3. 添加2D Pose Estimate工具（使用默认话题：/initialpose）");
    RCLCPP_INFO(this->get_logger(), "4. 添加Publish Point工具（使用默认话题：/clicked_point）");
    RCLCPP_INFO(this->get_logger(), "5. 添加PointCloud2显示（话题：/visual_obstacles）");
    RCLCPP_INFO(this->get_logger(), "6. 添加Path显示（话题：/visual_global_path和/visual_local_trajectory）");
}

// 初始化Ego Planner基础参数
void TrajectoryAndObstaclesPublisher::init_ego_planner_base()
{
    ego_planner_ = std::make_shared<PlannerInterface>();
    ego_planner_->initParam(max_vel_, max_acc_, max_jerk_);
    ego_planner_->initEsdfMap(
        map_x_size_, map_y_size_, map_z_size_,
        map_resolution_, map_origin_, map_inflate_value_
    );
}

// 统一添加障碍物函数
void TrajectoryAndObstaclesPublisher::add_obstacle_at_position(double x, double y)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    ObstacleInfo obs;
    obs.x = x;
    obs.y = y;
    obs.z = 0.0;
    
    obstacles_.push_back(obs);
    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "添加障碍物: (%.2f, %.2f), 总障碍物数量: %zu", 
                x, y, obstacles_.size());
}

// 2D Pose Estimate回调函数
void TrajectoryAndObstaclesPublisher::pose_estimate_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    add_obstacle_at_position(x, y);
    
    RCLCPP_INFO(this->get_logger(), "通过2D Pose Estimate添加障碍物: (%.2f, %.2f)", x, y);
}

// 触发规划话题回调
void TrajectoryAndObstaclesPublisher::trigger_plan_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (msg->data && !flag_) {
        if (has_valid_global_path_) {
            should_plan_ = true;
            needs_replan_ = true;  // 设置需要重新规划
            RCLCPP_INFO(this->get_logger(), "规划已触发! 路径点: %zu, 障碍物: %zu", 
                       global_plan_traj_.size(), obstacles_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "无法触发规划: 没有有效的全局路径!");
        }
        flag_ = msg->data;
    } else if(!msg->data && flag_){
        should_plan_ = false;
        needs_replan_ = false;  // 停止时清除重新规划标志
        obstacles_.clear();  // 清除障碍物
        global_plan_traj_.clear();  // 清除路径
        planned_traj.clear();  // 清除规划轨迹
        obstacles_.clear();  // 清除障碍物
        RCLCPP_INFO(this->get_logger(), "规划已停止.");
        flag_ = msg->data;
    }
}

// 处理2D Nav Goal - 生成从起点到目标的直线路
void TrajectoryAndObstaclesPublisher::goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 设置默认起点为(0,0)
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = msg->header;
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation = msg->pose.orientation;

    // 生成从起点到目标的路径
    generate_straight_path(start_pose, *msg);
    
    has_valid_global_path_ = true;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "生成从(0,0)到(%.2f, %.2f)的直线路径", 
                msg->pose.position.x, msg->pose.position.y);
}

// 生成直线路径
void TrajectoryAndObstaclesPublisher::generate_straight_path(const geometry_msgs::msg::PoseStamped& start, 
                                                           const geometry_msgs::msg::PoseStamped& goal)
{
    global_plan_traj_.clear();

    // 计算路径点数量（每0.1米一个点）
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    int num_points = std::max(2, static_cast<int>(distance / 0.1));

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double ratio = static_cast<double>(i) / (num_points - 1);
        PathPoint point;
        point.x = start.pose.position.x + ratio * dx;
        point.y = start.pose.position.y + ratio * dy;
        point.z = 0.0;
        global_plan_traj_.push_back(point);
    }
}

// // 处理Publish Point点击 - 添加障碍物
// void TrajectoryAndObstaclesPublisher::rviz_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
// {
//     add_obstacle_at_position(msg->point.x, msg->point.y);
// }

// 原有的全局路径回调
void TrajectoryAndObstaclesPublisher::rviz_global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "从RViz接收到空的全局路径!");
        has_valid_global_path_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    global_plan_traj_.clear();
    for (const auto& pose_stamped : msg->poses)
    {
        PathPoint path_point;
        path_point.x = pose_stamped.pose.position.x;
        path_point.y = pose_stamped.pose.position.y;
        path_point.z = pose_stamped.pose.position.z;
        global_plan_traj_.push_back(path_point);
    }

    has_valid_global_path_ = true;
    
    // 如果有路径更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "路径更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "从RViz接收到全局路径 (大小: %zu)", global_plan_traj_.size());
}

// 原有的障碍物点云回调
void TrajectoryAndObstaclesPublisher::rviz_obstacles_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg->width == 0)
    {
        RCLCPP_WARN(this->get_logger(), "从RViz接收到空的障碍物!");
        has_obstacles_ = false;
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    obstacles_.clear();
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        ObstacleInfo obs;
        obs.x = *iter_x;
        obs.y = *iter_y;
        obs.z = 0.0;
        obstacles_.push_back(obs);
    }

    has_obstacles_ = true;
    
    // 如果有障碍物更新且正在规划中，则标记需要重新规划
    if (should_plan_) {
        needs_replan_ = true;
        RCLCPP_INFO(this->get_logger(), "障碍物更新，已标记需要重新规划");
    }
    
    RCLCPP_INFO(this->get_logger(), "从RViz接收到障碍物 (数量: %zu)", obstacles_.size());
}

// 核心逻辑：检查数据更新→触发规划→发布结果
void TrajectoryAndObstaclesPublisher::publish_and_plan()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 如果有全局路径且应该规划，并且需要重新规划，则进行规划
    if (should_plan_ && has_valid_global_path_ && needs_replan_)
    {
        // 设置全局路径
        if (!global_plan_traj_.empty())
        {
            ego_planner_->setPathPoint(global_plan_traj_);
        }

        // 设置障碍物
        ego_planner_->setObstacles(obstacles_);

        // 触发Ego Planner规划
        ego_planner_->makePlan();
        
        if (has_obstacles_) {
            RCLCPP_INFO(this->get_logger(), "规划完成! 包含障碍物避让. 路径点: %zu, 障碍物: %zu", 
                       global_plan_traj_.size(), obstacles_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "规划完成! 无障碍物. 路径点: %zu", 
                       global_plan_traj_.size());
        }
        
        // 规划完成后，重置重新规划标志，但保持 should_plan_ 为 true
        // 这样下次有数据更新时可以自动重新规划
        needs_replan_ = false;
    }

    // 发布所有可视化数据（无论是否更新，保持实时显示）
    publish_global_path();
    publish_planned_trajectory();
    publish_obstacles();
}

// 发布可视化全局路径
void TrajectoryAndObstaclesPublisher::publish_global_path()
{
    // if (global_plan_traj_.empty()) return;

    nav_msgs::msg::Path visual_path;
    visual_path.header.stamp = this->now();
    visual_path.header.frame_id = "map";

    for (const auto& path_point : global_plan_traj_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_path.header;
        pose.pose.position.x = path_point.x;
        pose.pose.position.y = path_point.y;
        pose.pose.position.z = path_point.z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_path.poses.push_back(pose);
    }

    global_path_pub_->publish(visual_path);
}

// 发布Ego Planner规划后的局部轨迹
void TrajectoryAndObstaclesPublisher::publish_planned_trajectory()
{
    ego_planner_->getLocalPlanTrajResults(planned_traj);

    // if (planned_traj.empty()) return;

    nav_msgs::msg::Path visual_traj;
    visual_traj.header.stamp = this->now();
    visual_traj.header.frame_id = "map";

    for (size_t i = 0; i < planned_traj.size(); ++i)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = visual_traj.header;
        pose.pose.position.x = planned_traj[i].x;
        pose.pose.position.y = planned_traj[i].y;
        pose.pose.position.z = planned_traj[i].z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        visual_traj.poses.push_back(pose);
    }

    local_traj_pub_->publish(visual_traj);
}

// 发布可视化障碍物
void TrajectoryAndObstaclesPublisher::publish_obstacles()
{
    // if (obstacles_.empty()) return;

    sensor_msgs::msg::PointCloud2 visual_obs;
    visual_obs.header.stamp = this->now();
    visual_obs.header.frame_id = "map";
    visual_obs.width = obstacles_.size();
    visual_obs.height = 1;
    visual_obs.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(visual_obs);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(visual_obs, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(visual_obs, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(visual_obs, "z");

    for (const auto& obs : obstacles_)
    {
        *iter_x = static_cast<float>(obs.x);
        *iter_y = static_cast<float>(obs.y);
        *iter_z = 0.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    obs_pub_->publish(visual_obs);
}

// 主函数：启动节点
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryAndObstaclesPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}