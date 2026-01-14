
# Ego-Planner-ROS2 (早期版本 / Early Version)

> [!IMPORTANT]
> **版本重要更新与迁移提示：**
> *   **本项目状态**：这是早期版本，其核心优化器仍为 **3D 结构**。
> *   **实现方式**：在本项目中，我们通过从地图维度限制 Z 轴优化，使 3D 算法勉强适配地面 2D 移动机器人场景。
> *   **最新推荐**：为了获得更高的计算效率和真正的 2D 适配（算法层降维），请访问最新的重构版本：
> 👉 **[Ego-Planner-2D-ROS2 (最新 2D 原生版)](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)**
> 
> **新版优势**：从数学底层将优化器从 3D 降维至 2D，彻底消除 Z 轴冗余计算，更适合算力受限的嵌入式平台。

---

## 📝 项目简介
本项目是 Ego-Planner 算法在 ROS2 上的后端交互仿真实现。虽然核心优化器基于 3D 框架，但通过对地图信息的 Z 轴约束，实现了在地面 2D 移动机器人场景下的路径规划功能。

## 🛠️ 快速开始

### 1. 环境准备与编译
确保您的开发环境已安装 ROS2 (Foxy/Humble) 以及必要的 C++ 编译工具。
```bash
# 创建工作空间并克隆代码
mkdir -p ~/ego_planner_ws/src
cd ~/ego_planner_ws/src
git clone https://github.com/JackJu-HIT/Ego-Planner-ROS2.git

# 返回根目录编译
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 2. 运行规划节点
启动运动规划后端核心：
```bash
ros2 run ego_planner motion_plan
```

### 3. 可视化配置
1.  启动 **Rviz2**。
2.  在 Rviz2 中添加相应插件，配置并显示**全局轨迹 (Global Path)** 与 **障碍物信息 (Obstacles)**。

### 4. 触发规划任务
在 Rviz2 中设置好起点、目标点及环境障碍物后，通过发送以下指令触发规划逻辑：
```bash
ros2 topic pub /trigger_plan std_msgs/Bool "{data: true}" --once
```

---

## 📖 更多技术支持

欢迎关注我们的官方技术频道，深入了解机器人规划与控制的核心算法：

*   **微信公众号**：[机器人规划与控制研究所](https://mp.weixin.qq.com/s/kmslGiL9frpyzq-2575m5g)
    *   *提供详细的算法原理推导、参数调优及项目实战指南。*
*   **Bilibili 视频教程**：[机器人算法研究所](https://space.bilibili.com/384384547)
    *   *包含本项目及最新 2D 版本的仿真演示与代码讲解。*

---

## ⚖️ 开源协议
本项目遵循 [MIT](LICENSE) 开源协议。如果您觉得本项目对您的研究有帮助，欢迎给最新的 [Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2) 一个 **Star** ⭐！
