#!/usr/bin/env python3
"""
bridge_gz.py — Crazyflie/Gazebo ↔ MATLAB 桥接（原生 gz-transport 版）

与 bridge.py 功能完全相同，区别是 Gazebo 控制用原生 Python 绑定而非 subprocess。
每轮 control 耗时从 ~250ms 降到 ~10ms。

需要原生 gz Python 绑定：
  sudo apt install -y python3-gz-transport13 python3-gz-msgs10 python3-gz-sim8

用法:
  终端1: bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 1 -m crazyflie
  终端2: PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 bridge_gz.py
  终端3: matlab -r gz_pid
"""

import json
import socket
import sys
import threading
import time

import numpy as np

# gz-transport 原生绑定需要 /usr/lib/python3/dist-packages 在 path 中
GZ_PYTHON_PATH = "/usr/lib/python3/dist-packages"
if GZ_PYTHON_PATH not in sys.path:
    sys.path.insert(0, GZ_PYTHON_PATH)

from gz.transport13 import Node                           # noqa: E402
from gz.msgs10.world_control_pb2 import WorldControl      # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean                 # noqa: E402

from cflib.crtp import init_drivers                       # noqa: E402
from cflib.crazyflie import Crazyflie                     # noqa: E402
from cflib.crazyflie.log import LogConfig                 # noqa: E402
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie   # noqa: E402

# ==== 参数 ============================================================
STEPS = 50
WORLD = "crazysim_default"
URI = "udp://127.0.0.1:19850"
TCP_HOST = "127.0.0.1"
TCP_PORT = 15555

# ==== Gazebo 控制（原生 gz-transport，一次请求 ~1-5ms）===============
_node = None


def _get_node() -> Node:
    """懒初始化 gz-transport 节点（复用，不每次创建）。"""
    global _node
    if _node is None:
        _node = Node()
    return _node


def pause_sim():
    node = _get_node()
    req = WorldControl()
    req.pause = True
    rep = Boolean()
    node.request(f"/world/{WORLD}/control", req,
                 WorldControl, Boolean, timeout=500)


def step_n(n: int):
    """原子步进 n 步，完成后自动保持暂停。"""
    node = _get_node()
    req = WorldControl()
    req.pause = True
    req.multi_step = n
    rep = Boolean()
    node.request(f"/world/{WORLD}/control", req,
                 WorldControl, Boolean, timeout=500)


def unpause_sim():
    node = _get_node()
    req = WorldControl()
    req.pause = False
    rep = Boolean()
    node.request(f"/world/{WORLD}/control", req,
                 WorldControl, Boolean, timeout=500)


# ==== TCP 收发（与 bridge.py 相同）====================================
def send_msg(conn, obj: dict):
    conn.sendall((json.dumps(obj) + "\n").encode())


def recv_msg(conn, recv_buf: list) -> dict | None:
    while True:
        try:
            data = conn.recv(4096)
            if not data:
                return None
            recv_buf[0] += data.decode()
            while "\n" in recv_buf[0]:
                line, recv_buf[0] = recv_buf[0].split("\n", 1)
                line = line.strip()
                if line:
                    return json.loads(line)
        except socket.timeout:
            continue
        except json.JSONDecodeError as e:
            print(f"[bridge] JSON parse error: {e}")
            recv_buf[0] = ""
            return None


# ==== 主程序 ==========================================================
def main():
    # ── 连接 crazyflie ──
    print("[bridge] init cflib ...")
    init_drivers()
    time.sleep(0.3)

    print(f"[bridge] connect {URI} ...")
    scf = SyncCrazyflie(URI, cf=Crazyflie())
    scf.open_link()
    print("[bridge] connected")

    # ── 位姿日志 (100Hz) ──
    state = {"pos": np.zeros(3)}
    ev = threading.Event()

    lg = LogConfig(name="pose", period_in_ms=10)
    lg.add_variable("stateEstimate.x", "float")
    lg.add_variable("stateEstimate.y", "float")
    lg.add_variable("stateEstimate.z", "float")

    def cb(t, d, l):
        state["pos"] = np.array([d["stateEstimate.x"],
                                 d["stateEstimate.y"],
                                 d["stateEstimate.z"]])
        ev.set()

    scf.cf.log.add_config(lg)
    lg.data_received_cb.add_callback(cb)
    lg.start()

    # ── 等待初始位姿 ──
    print("[bridge] wait initial pose ...")
    ev.clear()
    if not ev.wait(timeout=2.0):
        print("[bridge]   no pose (sim paused?), stepping once to wake firmware ...")
        scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)
        ev.clear()
        step_n(1)
        if not ev.wait(timeout=2.0):
            print("[bridge] ERROR: still no pose after wake"); scf.close_link(); return
    cur = state["pos"].copy()
    print(f"[bridge] initial pose: "
          f"({cur[0]:.3f}, {cur[1]:.3f}, {cur[2]:.3f})")

    # ── 暂停仿真 ──
    print("[bridge] pause simulation ...")
    pause_sim()
    time.sleep(0.3)
    print("[bridge] paused. waiting for MATLAB ...")

    # ── TCP server ──
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)
    print(f"[bridge] TCP server listening on {TCP_HOST}:{TCP_PORT}")

    conn, addr = server.accept()
    print(f"[bridge] MATLAB connected from {addr}")
    conn.settimeout(10.0)

    buf = [""]
    it = 0
    t0 = time.time()

    try:
        while True:
            msg = recv_msg(conn, buf)
            if msg is None:
                break
            mtype = msg.get("type", "")

            if mtype == "get_state":
                cur = state["pos"].copy()
                send_msg(conn, {
                    "type": "state",
                    "x": float(cur[0]),
                    "y": float(cur[1]),
                    "z": float(cur[2]),
                })
                if it == 0:
                    print(f"[bridge] sent state to MATLAB")

            elif mtype == "control":
                vx = msg.get("vx", 0.0)
                vy = msg.get("vy", 0.0)
                vz = msg.get("vz", 0.0)
                yr = msg.get("yaw_rate", 0.0)

                scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yr)
                time.sleep(0.005)

                ev.clear()
                # 拆成 STEPS 次 step_n(1)，每次间隔 2ms
                # 让固件有足够 wall-clock 时间处理每步传感器数据
                for _ in range(STEPS):
                    step_n(1)
                #     # time.sleep(0.001)
                # step_n(50)

                if not ev.wait(timeout=2.0):
                    print("  [bridge] WARN: no pose after step")
                time.sleep(0.02)

                send_msg(conn, {"type": "step_done"})

                it += 1
                if it % 20 == 0:
                    cur = state["pos"]
                    print(f"[bridge] iter {it}  "
                          f"pos=({cur[0]:.3f},{cur[1]:.3f},{cur[2]:.3f})")

            else:
                print(f"[bridge] unknown msg type: {mtype}")

    except KeyboardInterrupt:
        print("\n[bridge] interrupted")
    finally:
        try:
            scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.005)
            step_n(10)
        except Exception:
            pass
        conn.close()
        server.close()
        print(f"[bridge] done  iters={it}  wall={time.time() - t0:.1f}s  "
              f"(sim paused, drone hovering)")
        scf.close_link()


if __name__ == "__main__":
    main()
