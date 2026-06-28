#!/usr/bin/env python3
"""基于 Pinocchio 的逆运动学 (IK) 模块。

针对 A-L1-GAMMA 7 自由度机械臂 (src/arm-platform/config/arm.urdf)。
采用阻尼最小二乘 (Damped Least Squares / Levenberg-Marquardt) 迭代求解。

说明:
- 该臂 joint6 与 joint7 轴线共线 (均为 x 轴), 末端姿态存在病态方向,
  因此默认只做 3D 位置 IK (7 关节冗余, 求解稳定);
  如需姿态, 可传入 target_rotation, 姿态误差以较低权重加入任务。
- 求解结果自动钳制到 URDF 关节限位内。

用法示例:
    from pinocchio_ik import PinocchioIK
    ik = PinocchioIK()
    q = ik.solve(np.array([0.4, 0.0, 0.2]))        # 仅位置
    pos, rot = ik.forward(q)                        # 正解验证
"""

import os

import numpy as np
import pinocchio as pin

def default_urdf_path() -> str:
    source_tree_urdf = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'arm.urdf')
    if os.path.exists(source_tree_urdf):
        return source_tree_urdf

    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('manipulator'), 'arm.urdf')
    except Exception:
        return source_tree_urdf


DEFAULT_URDF = default_urdf_path()

EE_FRAME = 'link7'  # 末端 frame 名


class PinocchioIK:
    # 从src/arm-platform/config/arm.urdf读取信息
    # Pinocchio 会根据 URDF 自动建立整条运动链。
    # 这个文件里没有手写 DH 参数，因为 Pinocchio 直接从 URDF 解析每个关节的位置、轴向、父子 link、关节限位。
    def __init__(self, urdf_path: str = DEFAULT_URDF, ee_frame: str = EE_FRAME):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.frame_id = self.model.getFrameId(ee_frame)
        self.nq = self.model.nq
        # URDF 关节限位
        self.q_lower = self.model.lowerPositionLimit
        self.q_upper = self.model.upperPositionLimit

    def forward(self, q: np.ndarray):
        """正解: 返回末端 (position(3,), rotation(3,3))。"""
        pin.framesForwardKinematics(self.model, self.data, q) # 让 Pinocchio 计算所有 link/frame 的位姿。
        oMf = self.data.oMf[self.frame_id] # origin/world/base 到 frame 的变换,这里的 frame 是 ee_frame 也就是 link7。
        return oMf.translation.copy(), oMf.rotation.copy()

    def solve(self,
              target_position: np.ndarray,
              target_rotation: np.ndarray = None,
              q_init: np.ndarray = None,
              max_iter: int = 200,
              eps: float = 1e-4,
              dt: float = 0.5,
              damp: float = 1e-6,
              rot_weight: float = 0.1):
        """求解 IK。

        Args:
            target_position: 目标位置 (3,), base_link 坐标系。
            target_rotation: 可选目标姿态 (3,3) 旋转矩阵; None 则只解位置。
            q_init: 初值 (nq,); None 则用零位。重复调用时传上次解可热启动。
            max_iter / eps: 迭代上限与位置误差收敛阈值 (m)。
            dt: 迭代步长。
            damp: 阻尼系数 (避免奇异)。
            rot_weight: 姿态任务权重 (相对位置任务)。

        Returns:
            (q, ok): q 为关节角 (nq,) 已钳制限位; ok 表示是否收敛。
        """
        q = pin.neutral(self.model) if q_init is None else q_init.copy()

        for _ in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            oMf = self.data.oMf[self.frame_id]

            # 位置误差 (world 系)
            err_pos = target_position - oMf.translation
            if target_rotation is not None:
                err_rot = pin.log3(target_rotation @ oMf.rotation.T)
                err = np.concatenate([err_pos, rot_weight * err_rot])
            else:
                err = err_pos

            if np.linalg.norm(err_pos) < eps:
                return np.clip(q, self.q_lower, self.q_upper), True

            # 末端 frame 雅可比 (LOCAL_WORLD_ALIGNED: 线速度部分即 world 系位置导数)
            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.frame_id,
                                          pin.LOCAL_WORLD_ALIGNED)
            if target_rotation is not None:
                J = np.vstack([J6[:3, :], rot_weight * J6[3:, :]])
            else:
                J = J6[:3, :]

            # 阻尼最小二乘: dq = J^T (J J^T + λI)^-1 err
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + damp * np.eye(JJt.shape[0]), err)
            q = pin.integrate(self.model, q, dq * dt)
            # 迭代中保持在限位内, 避免解跑飞
            q = np.clip(q, self.q_lower, self.q_upper)

        return q, np.linalg.norm(err_pos) < eps


if __name__ == '__main__':
    # 自测: 取一个可达位姿做 FK, 再用 IK 反求, 验证误差
    ik = PinocchioIK()
    print(f'模型: nq={ik.nq}, 末端 frame={EE_FRAME}')
    print(f'限位 lower: {np.round(ik.q_lower, 3)}')
    print(f'限位 upper: {np.round(ik.q_upper, 3)}')

    rng = np.random.default_rng(0)
    n_ok = 0
    for i in range(10):
        q_true = rng.uniform(ik.q_lower * 0.5, ik.q_upper * 0.5)
        p_target, _ = ik.forward(q_true)
        q_sol, ok = ik.solve(p_target)
        p_sol, _ = ik.forward(q_sol)
        e = np.linalg.norm(p_sol - p_target)
        n_ok += ok
        print(f'[{i}] target={np.round(p_target, 4)} err={e:.2e} ok={ok}')
    print(f'收敛 {n_ok}/10')
