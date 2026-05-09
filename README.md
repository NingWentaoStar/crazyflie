版权所有：PIControlLab
维护人：邓正宇、宁文涛

# Crazyflie 在环仿真 + Crazyswarm2 编队实验

本仓库包含 Crazyflie的gz-sim仿真、Crazyswarm2 控制后端、以及编队重配置（formation_reconfiguration）的完整实验链路。已适配到 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Sim。

## 1. 环境要求

| 组件 | 版本 |
|------|------|
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo | Gazebo Sim（Jazzy 对应版本） |
| Python | 3.12 |

## 2. 安装配置

### 2.1 Python 虚拟环境

```bash
cd ~/crazyflie
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install "empy<4" -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 2.2 编译 SITL 固件

```bash
cd ~/crazyflie/crazyflie_sim
source ~/crazyflie/.venv/bin/activate
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda | tr '\n' ':')
mkdir -p sitl_make/build && cd sitl_make/build
cmake ..
make -j"$(nproc)" all
```

### 2.3 ROS 工作区构建

```bash
cd ~/crazyflie/crazyswarm2_ws
source ~/crazyflie/.venv/bin/activate
colcon build --symlink-install
```

## 3. 标准运行顺序

### 3.1 启动仿真固件（SITL）

在 `crazyflie_sim` 目录下：

```bash
source ~/crazyflie/.venv/bin/activate
cd ~/crazyflie/crazyflie_sim
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 1 -m crazyflie
```

终端显示如下信息即为正常启动
```bash
[Msg] Init subs and Pubs done : 
[Msg] Received firmware handshake message...
```

### 3.2 启动 Crazyswarm2 后端

在 `crazyswarm2_ws` 目录下：

```bash
source ~/crazyflie/.venv/bin/activate
cd ~/crazyflie/crazyswarm2_ws
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib gui:=false mocap:=false teleop:=false rviz:=false
```
终端显示如下信息即为正常启动
```bash
[crazyflie_server]: All Crazyflies are fully connected!
```

### 3.3 停止运行

在 SITL 仿真终端按 `Ctrl+C`（脚本会自动清理 `cf2` 和 Gazebo），然后在后端终端按 `Ctrl+C`。

**一键清理所有残留进程：**
```bash
pkill -f crazyflie_server
pkill -x cf2
pkill -9 -f "gz sim"
```

## 其他说明

### TOC