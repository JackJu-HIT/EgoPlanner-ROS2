
# EgoPlanner-ROS2 (Early 3D Version)

> [!IMPORTANT]
> **版本重要更新提示：**
> 本仓库为该项目的**早期版本**，优化器核心仍基于 **3D** 环境设计，未进行降维处理。
> 
> **如果您是地面移动机器人用户，强烈建议访问最新重构版本：**
> 👉 **[Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)**
> 
> **最新版核心改进：**
> *   **算法降维**：将优化器从 3D 降至 2D，大幅减少计算开销。
> *   **平台适配**：真正适配地面移动机器人的运动约束，消除 Z 轴冗余。

---

## 📝 项目简介
本项目是 Ego-Planner 算法在 ROS2 上的后端交互仿真实现。此早期版本保留了 3D 规划能力，适用于无人机或需要三维空间避障的场景。

## 🛠️ 快速开始

### 1. 环境准备与编译
确保您已安装 ROS2 (Foxy/Humble) 及相关编译工具。
```bash
# 进入工作空间 src 目录
cd ~/your_ws/src
git clone https://github.com/JackJu-HIT/Ego-Planner-ROS2.git

# 返回根目录编译
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 2. 运行节点
启动运动规划后端节点：
```bash
ros2 run ego_planner motion_plan
```

### 3. 可视化与配置
1.  启动 **rviz2**。
2.  在 Rviz2 中配置并显示全局轨迹（Global Path）与障碍物信息（Obstacles）。

### 4. 触发规划任务
设置好起点、目标点及环境信息后，通过以下命令触发规划器：
```bash
ros2 topic pub /trigger_plan std_msgs/Bool "{data: true}" --once
```

---

## 📖 更多技术支持

欢迎关注我们的官方技术频道，获取更多关于机器人规划与控制的干货分享：

*   **微信公众号**：[机器人规划与控制研究所](https://mp.weixin.qq.com/s/kmslGiL9frpyzq-2575m5g)
    *   *深入浅出的算法解析与项目实战指南。*
*   **Bilibili 视频教程**：[机器人算法研究所](https://space.bilibili.com/你的UID)
    *   *直观的仿真演示与代码实现讲解。*

---

## ⚖️ 开源协议
本项目遵循 [MIT](LICENSE) 开源协议。如果您觉得本项目对您有帮助，欢迎给最新的 [2D 版本](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2) 一个 **Star** ⭐！
