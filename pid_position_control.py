#!/usr/bin/env python3
"""
PID 定点控制 — 先暂停仿真，每轮循环：
  ① 读当前位姿 → ② PID 算速度 → ③ 发速度指令 → ④ 严格 step 50 次

经验证：pause: true + multi_step: 50 组合可原子执行，一次调用完成。

终端1: bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 1 -m crazyflie
终端2: python3 pid_position_control.py
"""

import subprocess
import threading
import time

import numpy as np
from cflib.crtp import init_drivers
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ==== 参数 ============================================================
TARGET = np.array([0.0, 0.0, 1.0])
STEPS = 50               # 每轮严格 50 步 (50 × 0.001s = 0.05s)
SIM_DT = 0.05            # PID 控制周期
WORLD = "crazysim_default"
URI = "udp://127.0.0.1:19850"
MAX_ITER = 2000
CONVERGE = 0.03

# PID 增益（位置误差 → 期望速度）
Kp_xy, Ki_xy, Kd_xy = 0.6, 0.02, 0.15
Kp_z,  Ki_z,  Kd_z  = 0.8, 0.05, 0.2
MAX_V_XY, MAX_V_Z = 1.0, 0.5
I_MAX_XY, I_MAX_Z = 1.0, 1.0

# ==== Gazebo 控制 =====================================================
GZ_CMD = [
    "gz", "service",
    "-s", f"/world/{WORLD}/control",
    "--reqtype", "gz.msgs.WorldControl",
    "--reptype", "gz.msgs.Boolean",
    "--timeout", "3000",
    "--req",
]


def _gz(req: str):
    subprocess.run(GZ_CMD + [req], capture_output=True)


def pause():
    _gz("pause: true")


def step_n(n: int):
    """先暂停再步进 n 步，一次原子调用，步进后自动保持暂停。"""
    _gz(f"pause: true, multi_step: {n}")


def unpause():
    _gz("pause: false")


# ==== PID =============================================================
class PID:
    def __init__(self, target, dt):
        self.target = target
        self.dt = dt
        self.I = np.zeros(3)
        self.le = np.zeros(3)
        self.n = 0

    def __call__(self, pos):
        e = self.target - pos
        self.I += e * self.dt
        self.I[0] = np.clip(self.I[0], -I_MAX_XY, I_MAX_XY)
        self.I[1] = np.clip(self.I[1], -I_MAX_XY, I_MAX_XY)
        self.I[2] = np.clip(self.I[2], -I_MAX_Z, I_MAX_Z)
        d = (e - self.le) / self.dt if self.n else np.zeros(3)
        self.le = e.copy()
        self.n += 1
        Kp = np.array([Kp_xy, Kp_xy, Kp_z])
        Ki = np.array([Ki_xy, Ki_xy, Ki_z])
        Kd = np.array([Kd_xy, Kd_xy, Kd_z])
        v = Kp * e + Ki * self.I + Kd * d
        v[0] = np.clip(v[0], -MAX_V_XY, MAX_V_XY)
        v[1] = np.clip(v[1], -MAX_V_XY, MAX_V_XY)
        v[2] = np.clip(v[2], -MAX_V_Z, MAX_V_Z)
        return v, e


# ==== 主程序 ==========================================================
def main():
    # ── 连接 crazyflie ──
    print("[init] cflib ...")
    init_drivers()
    time.sleep(0.3)

    print(f"[init] connect {URI} ...")
    scf = SyncCrazyflie(URI, cf=Crazyflie())
    scf.open_link()
    print("[init] connected")

    # ── 位姿日志 (100Hz) ──
    P = {"v": np.zeros(3)}
    ev = threading.Event()

    lg = LogConfig(name="pose", period_in_ms=10)
    lg.add_variable("stateEstimate.x", "float")
    lg.add_variable("stateEstimate.y", "float")
    lg.add_variable("stateEstimate.z", "float")

    def cb(t, d, l):
        P["v"] = np.array([d["stateEstimate.x"],
                           d["stateEstimate.y"],
                           d["stateEstimate.z"]])
        ev.set()

    scf.cf.log.add_config(lg)
    lg.data_received_cb.add_callback(cb)
    lg.start()

    # ── 等待初始位姿（仿真还在跑，能收到 100Hz 位姿）──
    print("[init] wait initial pose ...")
    ev.clear()
    if not ev.wait(timeout=15):
        print("[ERROR] no initial pose"); scf.close_link(); return
    cur = P["v"].copy()
    print(f"[init] initial pose: ({cur[0]:.3f}, {cur[1]:.3f}, {cur[2]:.3f})")

    # ── 暂停仿真 ──
    print("[init] pause simulation ...")
    pause()
    time.sleep(0.3)
    print("[init] paused.\n")

    pid = PID(TARGET, SIM_DT)
    it = 0
    t0 = time.time()
    print(f"{'='*55}\n  target={TARGET}  每轮 {STEPS} 步 = {SIM_DT}s\n{'='*55}\n")

    try:
        while it < MAX_ITER:
            time.sleep(2)
            # ── ① 读位姿（上一轮 step 结束后的最新值）→ PID 算控制量 ──
            vel, err = pid(cur)
            dist = np.linalg.norm(err)

            # ── ② 打印 & 收敛检查 ──
            if it % 10 == 0:
                print(f"[{it:4d}] pos=({cur[0]:.3f},{cur[1]:.3f},{cur[2]:.3f})  "
                      f"err=({err[0]:.3f},{err[1]:.3f},{err[2]:.3f})  d={dist:.3f}")

            if dist < CONVERGE:
                print(f"\n  ✓ converged  pos={cur}  iters={it}")
                break

            # ── ③ 发速度指令（先发再步进）──
            scf.cf.commander.send_velocity_world_setpoint(
                float(vel[0]), float(vel[1]), float(vel[2]), 0.0)
            time.sleep(0.02)

            # ── ④ 清事件 → 原子步进 50 步 ──
            ev.clear()
            step_n(STEPS)

            # ── ⑤ 等新位姿 → 存 cur 给下一轮用 ──
            if not ev.wait(timeout=2.0):
                print("  [WARN] no pose after step")
            time.sleep(0.01)
            cur = P["v"].copy()

            it += 1

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        unpause()
        print(f"done  iters={it}  wall={time.time() - t0:.1f}s  "
              f"sim={it * SIM_DT:.1f}s")
        scf.close_link()


if __name__ == "__main__":
    main()
