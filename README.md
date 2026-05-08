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
mkdir -p crazyswarm2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/tools/build
cat > crazyswarm2_ws/src/crazyswarm2/crazyflie/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/tools/build/Findlibusb.cmake << 'CMAKE'
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)
find_path(libusb_INCLUDE_DIR NAMES libusb.h PATHS ${PC_LIBUSB_INCLUDE_DIRS})
find_library(libusb_LIBRARY NAMES usb-1.0 PATHS ${PC_LIBUSB_LIBRARY_DIRS})
set(LIBUSB_INCLUDE_DIR ${libusb_INCLUDE_DIR})
set(LIBUSB_LIBRARY ${libusb_LIBRARY})
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libusb REQUIRED_VARS libusb_INCLUDE_DIR libusb_LIBRARY)
if(NOT TARGET libusb)
  add_library(libusb UNKNOWN IMPORTED)
  set_target_properties(libusb PROPERTIES
    IMPORTED_LOCATION "${libusb_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${libusb_INCLUDE_DIR}"
  )
endif()
CMAKE
```

然后编译工作区：

```bash
cd ~/crazyflie/crazyswarm2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. 标准运行顺序

**启动顺序很重要，必须严格按以下步骤执行：**

### 3.1 启动仿真固件（SITL）

在 `crazyflie_sim` 目录下：

```bash
source ~/crazyflie/crazyflie_sim/.venv/bin/activate
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie
```

### 3.2 启动 Crazyswarm2 后端

在 `crazyswarm2_ws` 目录下：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib gui:=false mocap:=false teleop:=false rviz:=false
```