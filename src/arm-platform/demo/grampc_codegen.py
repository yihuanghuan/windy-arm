#!/usr/bin/env python3
"""为 GRAMPC LL100 NMPC 问题描述生成 CasADi C 导数函数 + 维度元信息。

GRAMPC 是连续时间 NMPC,状态 x=q∈R^nq、控制 u=q̇∈R^nq、动力学 ẋ=u。
本脚本复用 MpcPinocchioIK 的符号 FK / 避障 margin / 姿态误差,构造一组 CasADi
`Function`(运行/终端代价及其梯度、不等式约束及其 (dh/dx)^T·vec),用 CasADi
`CodeGenerator` 输出到 grampc_problem/ll100_grampc_gen.{c,h},供 C++ 问题类调用。

约定:
  - 参数 p = [目标位置(3), 目标旋转按行展开(9)] ∈ R^12。
  - 不等式 hfct(x,u,p) <= 0(GRAMPC 约定),由 -margin 组成(margin>=0 表示安全):
      关节限位 (q-q_upper, q_lower-q) 共 2*nq;前桨/机身避障 (-front, -body)。
  - dt 缩放并入权重(运行代价是连续时间积分,GRAMPC 自带 ∫dt;这里不再乘 dt)。

代价口径与 CasADi/ACADOS 后端保持一致(position/orientation/center/current 权重)。
joint_delta 项作用在 u=q̇ 上(连续时间速度惩罚),等价于离散 dq/dt 的平方惩罚。
"""

import json
import os
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
GEN_DIR = HERE / "grampc_problem"
GEN_BASENAME = "ll100_grampc_gen"
META_PATH = GEN_DIR / "ll100_grampc_meta.json"


def _build_mpc():
    """复用 benchmark 的 build_config 造一个开启避障的 MpcPinocchioIK。"""
    from benchmark_nmpc_solvers import build_config, solver_matrix
    from mpc_pinocchio_ik import MpcPinocchioIK

    grampc_sc = next(c for c in solver_matrix() if c.backend == "grampc")
    cfg, urdf = build_config(grampc_sc, "front_wing_arc")
    mpc = MpcPinocchioIK(urdf_path=urdf, ee_frame="F_Link", config=cfg)
    return mpc, cfg


def generate(force: bool = False) -> dict:
    """生成 C 代码 + 元信息 json。返回元信息 dict。

    若已存在且非 force,直接读元信息返回(供 benchmark 自动检测)。
    """
    GEN_DIR.mkdir(parents=True, exist_ok=True)
    c_path = GEN_DIR / f"{GEN_BASENAME}.c"
    if c_path.exists() and META_PATH.exists() and not force:
        return json.loads(META_PATH.read_text())

    import casadi as ca

    mpc, cfg = _build_mpc()
    nq = int(mpc.nq)
    nx = nq
    nu = nq
    np_param = 12  # 目标位置 3 + 目标旋转 9(按行展开)

    x = ca.MX.sym("x", nx)      # = q
    u = ca.MX.sym("u", nu)      # = q̇
    p = ca.MX.sym("p", np_param)
    xdes = ca.MX.sym("xdes", nx)  # 未使用,占位以匹配 GRAMPC 接口
    udes = ca.MX.sym("udes", nu)

    q = x
    p_pos = p[0:3]
    p_rot = ca.reshape(p[3:12], 3, 3).T  # 行展开 -> 3x3(reshape 默认列优先,故转置)

    pos = mpc._symbolic_fk.position(q)
    rot = mpc._symbolic_fk.rotation(q)
    pos_err = pos - p_pos
    rot_err = mpc._orientation_error_ca(rot, p_rot)

    q_center = ca.DM(mpc.q_center)
    # current_posture 项需要 x0;GRAMPC 没有把 x0 传进代价,按规格"二选一,简单优先"忽略。
    # 这样代价 = position + orientation + joint_delta(on u) + center_posture。
    l = (
        cfg.position_weight * ca.sumsqr(pos_err)
        + cfg.orientation_weight * ca.sumsqr(rot_err)
        + cfg.joint_delta_weight * ca.sumsqr(u)
        + cfg.center_posture_weight * ca.sumsqr(q - q_center)
    )
    dldx = ca.gradient(l, x)
    dldu = ca.gradient(l, u)

    V = (
        cfg.terminal_position_weight * ca.sumsqr(pos_err)
        + cfg.terminal_orientation_weight * ca.sumsqr(rot_err)
    )
    dVdx = ca.gradient(V, x)

    # 不等式约束 h(x,u,p) <= 0。
    h_terms = [q - ca.DM(mpc.q_upper), ca.DM(mpc.q_lower) - q]  # 关节限位
    fns, _m = mpc._obstacle_margin_functions()
    if fns is not None:
        margin = fns[0](q)  # margin >= 0 表示安全 -> 约束 -margin <= 0
        h_terms.append(-margin)
    h = ca.vertcat(*h_terms)
    nh = int(h.shape[0])

    vec = ca.MX.sym("vec", nh)
    # (dh/dx)^T * vec
    dhdx_vec = ca.jtimes(h, x, vec, True)

    # 组装 CasADi Function(签名固定,C++ 端按名调用)。
    f_l = ca.Function("ll100_l", [x, u, p, xdes, udes], [l])
    f_dldx = ca.Function("ll100_dldx", [x, u, p, xdes, udes], [dldx])
    f_dldu = ca.Function("ll100_dldu", [x, u, p, xdes, udes], [dldu])
    f_V = ca.Function("ll100_V", [x, p, xdes], [V])
    f_dVdx = ca.Function("ll100_dVdx", [x, p, xdes], [dVdx])
    f_h = ca.Function("ll100_h", [x, u, p], [h])
    f_dhdx_vec = ca.Function("ll100_dhdx_vec", [x, u, p, vec], [dhdx_vec])

    cg = ca.CodeGenerator(f"{GEN_BASENAME}.c", {"with_header": True})
    for fn in (f_l, f_dldx, f_dldu, f_V, f_dVdx, f_h, f_dhdx_vec):
        cg.add(fn)
    cwd = os.getcwd()
    try:
        os.chdir(GEN_DIR)
        cg.generate()
    finally:
        os.chdir(cwd)

    meta = {
        "nx": nx,
        "nu": nu,
        "np": np_param,
        "nh": nh,
        "nq": nq,
        "control_dt": float(cfg.control_dt),
        "horizon_steps": int(cfg.horizon_steps),
        "max_joint_step": float(cfg.max_joint_step),
        "gen_basename": GEN_BASENAME,
    }
    META_PATH.write_text(json.dumps(meta, indent=2))
    return meta


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="生成 GRAMPC LL100 CasADi C 导数代码")
    parser.add_argument("--force", action="store_true", help="强制重新生成")
    args = parser.parse_args()
    meta = generate(force=args.force)
    print("生成完成:", json.dumps(meta, indent=2))
    print("C 代码:", GEN_DIR / f"{GEN_BASENAME}.c")
