

# 📑 工程日志：阶段 2 - Gazebo 联合仿真的尝试与战略放弃

**日期：** 2025年12月
**状态：** ⚠️ 暂停/搁置 (Suspended)
**目标：** 试图引入 Gazebo 实现变构过程的 3D 可视化，以验证机械运动逻辑。

### 1. 3D 仿真环境搭建 (Setup Attempts)
*   **图形驱动攻坚：**
    *   **问题：** WSL2 运行 Gazebo 极其卡顿，检测发现使用的是 LLVMPipe (CPU软解)，且后续引发 `Segmentation fault` 驱动崩溃。
    *   **尝试：** 修改 `.bashrc` 环境变量，尝试了 `d3d12` 后端，最终降级为 `ogre` 渲染引擎才勉强稳定窗口。
*   **插件编译：**
    *   解决了 `libopencv` 和 `gstreamer` 的依赖缺失，成功编译 `ardupilot_gazebo` 插件。

### 2. 通信协议的泥潭 (The Protocol Deadlock)
这是导致本阶段未能完成的核心原因。
*   **TCP vs UDP 冲突：**
    *   **现象：** 使用 `--model JSON` (TCP) 启动 SITL 时，Gazebo 插件经常处于 UDP 监听状态，导致 `No JSON sensor message received` 报错。
    *   **尝试：** 修改 SDF 文件的 `lock_step` 和 `fdm_addr` 试图强制 TCP 握手，但引发了严重的 `Reset` 循环和时序死锁。
*   **端口绑定失败：**
    *   **现象：** 即使 `netstat` 显示端口在监听，SITL 依然无法建立稳定连接，或连接后无数据交换。

### 3. 模型文件 (SDF) 的开发与挫折
*   **手写模型：** 从零编写简易方块模型，因缺少 IMU 传感器定义导致飞控拒绝解锁。
*   **官方模型修改：**
    *   尝试引用 `iris_with_standoffs` 并添加关节，但遭遇命名空间冲突。
    *   最终尝试重写“扁平化”模型，添加了 `cmd_max` (扭矩限制) 和 `p_gain`，试图解决“有信号但关节不动”的问题。

### ⏹ 阶段决策 (Decision Making)
尽管修复了部分模型错误，但 **WSL2 + Gazebo + ArduPilot** 的工具链维护成本过高，网络与驱动的不稳定性严重拖慢了核心算法的开发进度。
*   **结论：** **果断放弃 Gazebo 3D 仿真路线**。
*   **转向：** 回归纯 SITL 环境，通过数据曲线而非 3D 画面来验证物理逻辑。

---


