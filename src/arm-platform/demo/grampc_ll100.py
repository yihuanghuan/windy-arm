#!/usr/bin/env python3
"""GRAMPC 求解器 LL100 NMPC IK 运行器,接口对齐 MpcPinocchioIK.step。

GRAMPC 用增广拉格朗日 + 投影梯度法求解连续时间最优控制:
  状态 x=q∈R^nq,控制 u=q̇∈R^nq,ẋ=u;参数 p=[目标位置3, 目标旋转按行9]。
  代价 / 约束由 CasADi 生成的 C 函数提供(见 grampc_codegen.py / ll100_problem.cpp)。

GrampcRunner 内部:
  - 持有一个 MpcPinocchioIK(仅用于 FK / 避障诊断 / 奇异度量,复用同一套运动学)。
  - 加载编译好的 ll100_grampc_problem 模块,配置 pygrampc.Grampc。
  - step(targets, q0, warm_start, target_rotation) -> (q_next, ok, info),
    与 benchmark 调用 mpc.step 完全一致;q_next = q0 + u0 * control_dt。
"""

import os
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np


class GrampcRunner:
    """对齐 MpcPinocchioIK.step 口径的 GRAMPC 求解器封装。"""

    def __init__(self, urdf_path: str, ee_frame: str = "F_Link", config=None):
        from mpc_pinocchio_ik import MpcPinocchioIK
        import grampc_build
        import grampc_codegen
        from pygrampc import Grampc

        # 复用运动学 / 避障 / 诊断。这个实例本身不会被用来求解(只用 FK 等)。
        self.mpc = MpcPinocchioIK(urdf_path=urdf_path, ee_frame=ee_frame, config=config)
        self.config = self.mpc.config
        self.nq = self.mpc.nq

        meta = grampc_codegen.generate()
        problem_mod = grampc_build.ensure_module()
        self.meta = meta
        self.problem = problem_mod.LL100Problem(
            int(meta["nx"]), int(meta["nu"]), int(meta["np"]), int(meta["nh"])
        )

        self._control_dt = float(self.config.control_dt)
        horizon = int(self.config.horizon_steps)
        # 时域 T = horizon_steps * control_dt;Nhor 取 max(20, horizon+1)。
        self._Thor = horizon * self._control_dt
        nhor = max(20, horizon + 1)
        # 最大关节速度 = max_joint_step / control_dt(箱约束 ±umax)。
        umax = float(self.config.max_joint_step) / self._control_dt

        self.grampc = Grampc(self.problem, None)
        self.grampc.set_param({
            "Thor": self._Thor,
            "dt": self._control_dt,
            "x0": np.zeros(self.nq),
            "u0": np.zeros(self.nq),
            "umax": np.full(self.nq, umax),
            "umin": np.full(self.nq, -umax),
            "p0": np.zeros(int(meta["np"])),
            "t0": 0.0,
        })
        # 目标前瞻索引:默认 2 = 瞄前方约一个时域长度(offsets[2]=preview_stride*2≈horizon 步)。
        # 实测该值稳态 RMS 0.26mm(原 -1 瞄最远 28 路点 → lead 偏差 8.3mm)。env 可覆盖;
        # 注:该最优索引与 horizon_steps/preview_stride 绑定,换基线需重扫。
        self._target_index = int(os.environ.get("GRAMPC_TARGET_INDEX", "2"))
        max_grad_iter = int(os.environ.get("GRAMPC_MAX_GRAD_ITER", "3"))
        max_mult_iter = int(os.environ.get("GRAMPC_MAX_MULT_ITER", "2"))
        self.grampc.set_opt({
            "Nhor": nhor,
            "MaxGradIter": max_grad_iter,
            "MaxMultIter": max_mult_iter,
            "ShiftControl": "on",
            "Integrator": "heun",
            "IntegralCost": "on",
            "TerminalCost": "on",
            "InequalityConstraints": "on",
            "EqualityConstraints": "off",
            "TerminalInequalityConstraints": "off",
            "TerminalEqualityConstraints": "off",
            "ConstraintsHandling": "auglag",
            "ConstraintsAbsTol": np.full(int(meta["nh"]), 1e-3),
            "ConvergenceCheck": "off",
            # 增广拉格朗日惩罚参数。固定取值;不调用 estim_penmin——本问题尺度下
            # estim_penmin 会算出一个 GRAMPC C 端拒绝的 PenaltyMin(直接报错退出)。
            "PenaltyMin": 1.0,
            "PenaltyMax": 1e6,
            "PenaltyIncreaseFactor": 1.05,
            "PenaltyDecreaseFactor": 0.95,
            "MultiplierMax": 1e6,
        })

        self._initialized = False

    # ---- 与 MpcPinocchioIK 共用的小工具(代理到内部 mpc) ----
    def forward(self, q):
        return self.mpc.forward(q)

    @property
    def model(self):
        return self.mpc.model

    @property
    def data(self):
        return self.mpc.data

    @property
    def frame_id(self):
        return self.mpc.frame_id

    @property
    def q_lower(self):
        return self.mpc.q_lower

    @property
    def q_upper(self):
        return self.mpc.q_upper

    def _obstacle_diagnostics(self, plan):
        return self.mpc._obstacle_diagnostics(plan)

    # 障碍物几何访问器:代理到内部 MpcPinocchioIK,供 ROS 节点画 marker / 建路径复用。
    def obstacle_disks(self):
        return self.mpc.obstacle_disks()

    def body_obstacle_boxes(self):
        return self.mpc.body_obstacle_boxes()

    def capsule_sample_positions(self, q):
        return self.mpc.capsule_sample_positions(q)

    def _position_error(self, q, target) -> float:
        pos, _ = self.mpc.forward(q)
        return float(np.linalg.norm(pos - target))

    def step(
        self,
        target_position: np.ndarray,
        q_current: np.ndarray,
        warm_start: Optional[np.ndarray] = None,
        target_rotation: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool, Dict[str, object]]:
        """单步滚动 MPC,返回 (q_next, ok, info),口径与 MpcPinocchioIK.step 一致。"""
        q0 = np.clip(np.asarray(q_current, dtype=float).reshape(self.nq),
                     self.q_lower, self.q_upper)
        targets = self.mpc._target_sequence(target_position)
        rotations, orientation_active = self.mpc._rotation_sequence(target_rotation, q0)

        # 目标取前瞻序列的 _target_index 点(默认 2=前方约一个时域;瞄太远 lead 偏差大)。
        ti = self._target_index
        p_pos = np.asarray(targets[ti], dtype=float).reshape(3)
        p_rot = np.asarray(rotations[ti], dtype=float).reshape(3, 3)
        p_vec = np.concatenate([p_pos, p_rot.reshape(-1)])

        # warm start:首次设 x0;之后靠 GRAMPC 自身 ShiftControl 滚动热启动。
        self.grampc.set_param({"x0": q0, "p0": p_vec})

        # GRAMPC 一次 run 是一次完整(多梯度/多乘子)迭代。
        cpu_ms = float(self.grampc.run())
        solve_time = cpu_ms / 1000.0

        u0 = np.asarray(self.grampc.sol.unext, dtype=float).reshape(self.nq)
        status = int(self.grampc.sol.status)
        try:
            nit = int(np.sum(self.grampc.sol.iter))
        except Exception:
            nit = 0

        q_next = np.clip(q0 + u0 * self._control_dt, self.q_lower, self.q_upper)

        # 预测轨迹:GRAMPC 预测 Nhor 个状态点(rws.x 形状 nx x Nhor),
        # 取后续 horizon_steps 个(去掉第 0 个 = 当前)作为 N×nq 轨迹,供 warm_start 与诊断。
        horizon = int(self.config.horizon_steps)
        try:
            pred_x = np.asarray(self.grampc.rws.x, dtype=float)  # nx x Nhor
            traj = pred_x.T  # Nhor x nx
            if traj.shape[0] >= horizon + 1:
                trajectory = traj[1:horizon + 1, :].copy()
            else:
                # 不够长就重复末点补齐。
                pad = np.repeat(traj[-1:, :], horizon - (traj.shape[0] - 1), axis=0)
                trajectory = np.vstack([traj[1:, :], pad])[:horizon]
        except Exception:
            trajectory = np.repeat(q_next.reshape(1, self.nq), horizon, axis=0)
        trajectory = np.clip(trajectory, self.q_lower, self.q_upper)

        terminal_error = self._position_error(trajectory[-1], targets[-1])
        first_error = self._position_error(q_next, targets[0])
        # 约束可行性:用实际即将执行的 q_next 检查避障 + 限位是否满足。
        plan_feasible = self._plan_feasible(q_next, trajectory)
        tracking_ok = terminal_error <= self.config.success_tolerance
        ok = bool(tracking_ok and plan_feasible)

        # status==0 表示 GRAMPC 正常;命令始终可用(梯度法总给出一个可执行 u)。
        success = (status == 0)
        command_usable = True

        info: Dict[str, object] = {
            "solve_time": solve_time,
            "terminal_error": terminal_error,
            "first_error": first_error,
            "trajectory": trajectory,
            "status": 0 if success else 1,
            "success": success,
            "command_usable": command_usable,
            "nit": nit,
            "orientation_active": orientation_active,
            "plan_feasible": plan_feasible,
            "tracking_ok": tracking_ok,
        }
        self._initialized = True
        return q_next, ok, info

    def _plan_feasible(self, q_next: np.ndarray, trajectory: np.ndarray) -> bool:
        """限位 + 避障可行性检查(对实际将执行的 q_next)。"""
        if np.any(q_next < self.q_lower - 1e-6) or np.any(q_next > self.q_upper + 1e-6):
            return False
        try:
            diag = self.mpc._obstacle_diagnostics(q_next.reshape(1, self.nq))
            viol = diag.get("obstacle_violation_max")
            if viol is not None and float(viol) > 1e-4:
                return False
        except Exception:
            pass
        return True


def build_runner_from_config(config, urdf_path: str, ee_frame: str = "F_Link") -> GrampcRunner:
    return GrampcRunner(urdf_path=urdf_path, ee_frame=ee_frame, config=config)
