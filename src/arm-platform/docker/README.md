# Docker 编译验证指南

本目录包含用于验证 manipulator 包编译的 Dockerfile。

## Dockerfile 说明

### 1. Dockerfile.build
完整版本的 Dockerfile，包含所有必要的构建工具和依赖。

### 2. Dockerfile.verify
精简版本的 Dockerfile，专门用于快速编译验证，构建完成后会自动验证编译结果。

## 使用方法

### 方法一：使用 Dockerfile.verify（推荐用于快速验证）

```bash
# 在项目根目录执行
cd /home/iusl/huaben_ws/src/arm-platform

# 构建镜像
docker build -f docker/Dockerfile.verify -t manipulator:verify .

# 运行容器验证编译
docker run --rm manipulator:verify
```

### 方法二：使用 Dockerfile.build

```bash
# 构建镜像
docker build -f docker/Dockerfile.build -t manipulator:build .

# 运行容器
docker run -it manipulator:build
```

### 方法三：交互式调试

```bash
# 构建镜像
docker build -f docker/Dockerfile.build -t manipulator:build .

# 运行容器并进入交互式 shell
docker run -it --rm manipulator:build /bin/bash

# 在容器内手动构建
cd /ros_ws
colcon build --packages-select manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 预期输出

如果编译成功，您应该看到：

1. 构建过程没有错误
2. 生成的可执行文件：
   - `arm_hardware_node`
   - `master_arm_node`
   - `slave_arm_node`
3. 生成的库文件：
   - `libmanipulator_core.so`
   - `libgravity_compensation.so`

## 故障排除

### 常见问题

1. **依赖缺失**
   ```bash
   # 在容器内手动安装缺失的依赖
   apt-get update
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **编译错误**
   ```bash
   # 查看详细编译日志
   colcon build --packages-select manipulator --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+
   ```

3. **权限问题**
   ```bash
   # 确保有正确的文件权限
   chmod -R 755 /ros_ws/src
   ```

## 清理

```bash
# 删除构建的镜像
docker rmi manipulator:verify manipulator:build

# 清理所有悬空镜像
docker image prune -f
```

## 注意事项

1. Docker 镜像基于 ROS 2 Humble，确保您的代码与 Humble 兼容
2. 首次构建可能需要较长时间下载依赖
3. **Serial 包依赖**：Dockerfile 会自动从 GitHub 克隆 `https://github.com/ZhaoXiangBox/serial.git` 作为串口通信依赖
4. **dummy-interface 包依赖**：Dockerfile 会自动从 GitLab 克隆 `http://10.0.2.66:8000/aerial_manipulator_hub/interface/dummy-interface.git` 作为接口依赖
5. 构建过程需要约 1-2GB 的磁盘空间

## 依赖说明

### Serial 包
- **来源**：https://github.com/ZhaoXiangBox/serial
- **用途**：提供串口通信功能
- **安装方式**：Dockerfile 构建时克隆到 ROS 2 工作空间，使用 colcon 与 manipulator 一起编译
- **编译方式**：作为 ROS 2 包使用 colcon 编译
  ```bash
  # serial 包会被克隆到工作空间 src 目录
  # 然后使用 colcon 一起编译
  colcon build --packages-select serial dummy-interface manipulator
  ```
- **版本**：ROS2 Foxy/Humble 兼容版本

### Dummy-Interface 包
- **来源**：http://10.0.2.66:8000/aerial_manipulator_hub/interface/dummy-interface.git
- **用途**：提供机械臂控制接口定义
- **安装方式**：Dockerfile 构建时克隆到 ROS 2 工作空间，使用 colcon 与 manipulator 一起编译
- **编译方式**：作为 ROS 2 包使用 colcon 编译
  ```bash
  # dummy-interface 包会被克隆到工作空间 src 目录
  # 然后使用 colcon 一起编译
  colcon build --packages-select serial dummy-interface manipulator
  ```

### 其他依赖
- **ROS 2 包**：rclcpp, sensor_msgs, std_msgs, geometry_msgs
- **第三方库**：Eigen3, serial
