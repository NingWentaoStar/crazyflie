#!/usr/bin/env python3
"""
bridge.py — Crazyflie/Gazebo ↔ MATLAB 桥接

职能：
  1. 连接 crazyflie，暂停 gz-sim
  2. 作为 TCP server 等待 MATLAB 连接
  3. 协议（JSON，换行分隔，端口 15555）：
       MATLAB → bridge:  {"type": "get_state"}
       bridge → MATLAB:  {"type": "state", "x": 0.0, "y": 0.0, "z": 0.0}
       MATLAB → bridge:  {"type": "control", "vx": 0.1, "vy": 0.0, "vz": 0.5, "yaw_rate": 0.0}
       bridge:           发速度 → step 50 → 读新位姿
       bridge → MATLAB:  {"type": "step_done"}

用法:
  终端1: bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 1 -m crazyflie
  终端2: python3 bridge.py          # 先启动，等待 MATLAB
  终端3: matlab -r gz_pid           # 后启动，连接 bridge
"""

import json
import socket
import subprocess
import threading
import time

import numpy as np
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ==== 参数 ============================================================
STEPS = 50                       # 每轮步进数
WORLD = "crazysim_default"
URI = "udp://127.0.0.1:19850"
TCP_HOST = "127.0.0.1"
TCP_PORT = 15555

# ==== Gazebo 控制 =====================================================
GZ_CMD = [
    "gz", "service",
    "-s", f"/world/{WORLD}/control",
    "--reqtype", "gz.msgs.WorldControl",
    "--reptype", "gz.msgs.Boolean",
    "--timeout", "500",            # 50 步只需 0.05s 仿真，500ms 足够，避免 3s 超时
    "--req",
]


def _gz(req: str):
    # DEVNULL 替代 capture_output，避免 pipe 读写开销
    subprocess.run(GZ_CMD + [req],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def pause_sim():
    _gz("pause: true")


def step_n(n: int):
    """原子步进 n 步，完成后自动保持暂停。"""
    _gz(f"pause: true, multi_step: {n}")


def unpause_sim():
    _gz("pause: false")


# ==== TCP 收发 ========================================================
def send_msg(conn, obj: dict):
    """发送 JSON 消息（换行分隔）。"""
    conn.sendall((json.dumps(obj) + "\n").encode())


def recv_msg(conn, recv_buf: list) -> dict | None:
    """接收一行完整 JSON 消息。阻塞。"""
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

    # ── 等待初始位姿（兼容"仿真在跑"和"上次退出时暂停了"两种情况）──
    print("[bridge] wait initial pose ...")
    ev.clear()
    if not ev.wait(timeout=2.0):
        # 可能仿真被上次 bridge 退出时保持暂停了，步进一帧唤醒飞控
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

    buf = [""]   # mutable recv buffer
    it = 0
    t0 = time.time()

    try:
        while True:
            # ── 等 MATLAB 请求 ──
            msg = recv_msg(conn, buf)
            if msg is None:
                break
            mtype = msg.get("type", "")

            if mtype == "get_state":
                # 读取当前位姿 → 回传
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
                # 发速度 → 步进 50 → 读新位姿 → 告诉 MATLAB 完成
                vx = msg.get("vx", 0.0)
                vy = msg.get("vy", 0.0)
                vz = msg.get("vz", 0.0)
                yr = msg.get("yaw_rate", 0.0)

                scf.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yr)
                time.sleep(0.005)   # UDP localhost 亚毫秒，5ms 足够

                ev.clear()
                step_n(STEPS)

                if not ev.wait(timeout=2.0):
                    print("  [bridge] WARN: no pose after step")
                # 等剩余日志回调到位（~5 次回调，第一次触发 ev，后续 ~50μs 内到）
                time.sleep(0.03)

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
        # 发零速度 + 步进几帧，让飞控在暂停前真正执行悬停指令
        try:
            scf.cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.005)
            step_n(10)  # 步进 10 帧执行零速度
        except Exception:
            pass
        conn.close()
        server.close()
        print(f"[bridge] done  iters={it}  wall={time.time() - t0:.1f}s  "
              f"(sim paused, drone hovering)")
        scf.close_link()


if __name__ == "__main__":
    main()
