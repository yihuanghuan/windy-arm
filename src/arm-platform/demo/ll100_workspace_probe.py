#!/usr/bin/env python3
"""LL100-LH420 避障可达范围探测工具。

两阶段探测"末端竖直朝下 + 无奇异 + 与桨盘/机身无碰撞"的安全可达区:
1. 关节空间蒙特卡洛粗扫(全局形状);
2. 前方 ROI 逐体素 IK 精查——蒙特卡洛在笛卡尔空间不均匀,"折叠着往前下方伸+朝下"
   的构型是 6 维里极薄的流形,随机采样几乎砸不中(空 ≠ 不可达!demo 的 front_wing_arc
   就在采样空区里正常跑)。对 MC 没填上的体素逐个跑 DLS-IK(多 yaw 目标+多种子),
   成功再过奇异+碰撞筛,才能给出"这个格子到底可不可达"的真实答案。

姿态判据 = 末端 x 轴与 -Z 的夹角 ≤ 容差(绕竖直轴的 yaw 自由)——与 demo 的软姿态锁
语义一致;不是"整个旋转矩阵全锁"(那样会把合法构型多砍 ~9 成)。

碰撞模型与主 demo 完全一致(复用同一套 capsule_points / 障碍几何 / margin),
所以吃到用户在 move_ll100_lh420_nmpc_demo.py 里微调过的小球位置。

手动启动(先起控制版 launch 提供 RViz + robot_state_publisher):
    python3 ll100_workspace_probe.py
然后在 RViz Add → By topic → /ll100_workspace/reachable(和 /ll100_workspace/obstacles)。
"""

import math

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from mpc_pinocchio_ik import (
    DEFAULT_LL100_URDF,
    MpcIkConfig,
    MpcPinocchioIK,
    _box_signed_distance_np,
    _disk_signed_distance_np,
)
from move_ll100_lh420_nmpc_demo import (
    BASE_FRAME,
    CAPSULE_CENTERLINE_POINTS,
    DEFAULT_CAPSULE_JOINT_RADII,
    DEFAULT_CAPSULE_LINK_RADII,
    DEFAULT_LL100_BODY_OBSTACLE_BOXES,
    LL100_NMPC_URDF,
    _quat_from_z_axis,
)
from pinocchio_ik import PinocchioIK

LINK_FRAMES = ("A_Link", "B_Link", "C_Link", "D_Link", "E_Link", "F_Link")
# 竖直朝下目标姿态,与 demo 的 _orientation_target_rotation("vertical_down") 完全一致(EE x 轴指 -Z)。
VERTICAL_DOWN = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=float,
)


class LL100WorkspaceProbe(Node):
    def __init__(self):
        super().__init__("ll100_workspace_probe")

        # 轴锥判据下命中率 ~3%,30 万采样(几秒)够全局粗扫;局部细节靠 grid-IK 精查,不靠堆采样。
        self.num_samples = int(self.declare_parameter("num_samples", 300000).value)
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.02).value)
        self.manip_threshold = float(self.declare_parameter("manipulability_threshold", 0.012).value)
        self.orientation_tol_deg = float(self.declare_parameter("orientation_tol_deg", 20.0).value)
        self.require_vertical_down = bool(self.declare_parameter("require_vertical_down", True).value)
        self.extra_clearance = float(self.declare_parameter("extra_clearance", 0.0).value)
        self.seed = int(self.declare_parameter("seed", 0).value)
        # 前方 ROI 逐体素 IK 精查:MC 采不到的薄流形区(前下方折叠构型)靠它填真值。
        self.grid_ik_enabled = bool(self.declare_parameter("grid_ik_enabled", True).value)
        # ROI = [xmin,xmax,ymin,ymax,zmin,zmax],base_link 系,默认盖住机身前方工作区(含 front_wing_arc)。
        self.grid_ik_region = [float(v) for v in self.declare_parameter(
            "grid_ik_region", [0.40, 0.78, -0.35, 0.35, -0.32, 0.08]).value]
        self.grid_ik_yaw_count = int(self.declare_parameter("grid_ik_yaw_count", 3).value)  # 每体素尝试的 yaw 目标数
        # 碰撞 margin 默认与主 demo 一致;单独暴露以便按需收紧/放松。
        self.obstacle_safety_margin = float(self.declare_parameter("obstacle_safety_margin", 0.005).value)
        self.body_obstacle_margin = float(self.declare_parameter("body_obstacle_margin", 0.005).value)

        cfg = MpcIkConfig(
            solver_backend="acados",  # 只用 FK + numpy 碰撞,不 solve/编译 acados
            obstacle_avoidance_enabled=True,
            obstacle_safety_margin=self.obstacle_safety_margin,
            obstacle_disk_half_thickness=0.02,
            obstacle_ll100_urdf_path=DEFAULT_LL100_URDF,
            obstacle_coordinate_mode="ll100_base",
            obstacle_keypoint_frames=LINK_FRAMES,
            body_obstacle_enabled=True,
            body_obstacle_margin=self.body_obstacle_margin,
            body_obstacle_boxes=DEFAULT_LL100_BODY_OBSTACLE_BOXES,
            capsule_radius=0.018,
            capsule_joint_radii=tuple(DEFAULT_CAPSULE_JOINT_RADII),
            capsule_link_radii=tuple(DEFAULT_CAPSULE_LINK_RADII),
            capsule_points=CAPSULE_CENTERLINE_POINTS,  # 与主 demo 同一组中心线小球
            capsule_samples_per_segment=3,
            front_obstacle_capsule=True,
            capsule_frame_sequence=LINK_FRAMES,
        )
        self.mpc = MpcPinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link", config=cfg)
        # grid-IK 精查用的 DLS 求解器(带限位钳制/阻尼,支持目标姿态),与 mpc 共享同一 URDF/末端。
        self.ik = PinocchioIK(urdf_path=LL100_NMPC_URDF, ee_frame="F_Link")

        # 缓存不随 q 变的量
        self.sample_radii = np.asarray(self.mpc.capsule_sample_radii(), dtype=float)
        self.disks = self.mpc.obstacle_disks()
        self.boxes = self.mpc.body_obstacle_boxes()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.reachable_pub = self.create_publisher(MarkerArray, "/ll100_workspace/reachable", latched)
        self.obstacle_pub = self.create_publisher(MarkerArray, "/ll100_workspace/obstacles", latched)

        self.get_logger().info(
            f"开始探测可达区: num_samples={self.num_samples}, voxel={self.voxel_size*1000:.0f}mm, "
            f"manip>={self.manip_threshold:.4f}, 竖直朝下={self.require_vertical_down}"
            f"(容差{self.orientation_tol_deg:.0f}°), extra_clearance={self.extra_clearance*1000:.0f}mm, "
            f"碰撞球数={len(self.sample_radii)}, 桨盘={len(self.disks)}, 机身box={len(self.boxes)}"
        )
        # 先发障碍和 MC 粗扫结果(秒级),IK 精查(分钟级)在后面补充刷新——RViz 不用干等。
        self._obstacle_markers = self._build_obstacle_markers()
        self._reachable_markers = MarkerArray()
        self._reachable_markers = self._compute_and_build()

        # latched 已保证晚加入的 RViz 也能收到;定时器兜底重发。
        self.timer = self.create_timer(2.0, self._publish)
        self._publish()

    # ---- 计算 ----
    def _joint_bounds(self):
        lo = np.asarray(self.mpc.q_lower, dtype=float).copy()
        hi = np.asarray(self.mpc.q_upper, dtype=float).copy()
        # 无限位关节兜底成 [-pi, pi],避免 uniform 抽到 inf。
        lo = np.where(np.isfinite(lo), lo, -math.pi)
        hi = np.where(np.isfinite(hi), hi, math.pi)
        return lo, hi

    def _collision_free(self, samples: np.ndarray) -> bool:
        extra = self.extra_clearance
        sm = self.obstacle_safety_margin
        bm = self.body_obstacle_margin
        for pos, r in zip(samples, self.sample_radii):
            for disk in self.disks:
                if _disk_signed_distance_np(pos, disk) - r - sm < extra:
                    return False
            for box in self.boxes:
                if _box_signed_distance_np(pos, box) - r - bm < extra:
                    return False
        return True

    def _orientation_ok(self, R: np.ndarray, cos_tol: float) -> bool:
        """轴锥判据:末端 x 轴(R 第 0 列)与 -Z 的夹角 ≤ 容差,绕竖直轴 yaw 自由(与 demo 软锁语义一致)。"""
        return float(-R[2, 0]) >= cos_tol

    def _manipulability(self, q: np.ndarray) -> float:
        jac = pin.computeFrameJacobian(
            self.mpc.model, self.mpc.data, q, self.mpc.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, :]
        return math.sqrt(max(float(np.linalg.det(jac @ jac.T)), 0.0))

    def _compute_and_build(self) -> MarkerArray:
        mpc = self.mpc
        lo, hi = self._joint_bounds()
        rng = np.random.default_rng(self.seed)
        cos_tol = math.cos(math.radians(self.orientation_tol_deg))
        vs = self.voxel_size

        voxels: dict = {}
        passers: list = []  # (pos, q) —— 给 grid-IK 当种子
        n_ori = n_manip = n_free = 0
        log_every = max(1, self.num_samples // 10)

        for i in range(self.num_samples):
            if i % log_every == 0 and i > 0:
                self.get_logger().info(
                    f"  MC 进度 {100*i//self.num_samples}%  过姿态={n_ori} 过奇异={n_manip} 过碰撞={n_free} 体素={len(voxels)}"
                )
            q = rng.uniform(lo, hi)
            pos, R = mpc.forward(q)

            if self.require_vertical_down and not self._orientation_ok(R, cos_tol):
                continue
            n_ori += 1

            manip = self._manipulability(q)
            if manip < self.manip_threshold:
                continue
            n_manip += 1

            samples = mpc.capsule_sample_positions(q)
            if not self._collision_free(samples):
                continue
            n_free += 1
            passers.append((pos.copy(), q.copy()))

            key = (
                int(math.floor(pos[0] / vs)),
                int(math.floor(pos[1] / vs)),
                int(math.floor(pos[2] / vs)),
            )
            if key not in voxels or manip > voxels[key]:
                voxels[key] = manip

        # MC 粗扫先上屏(latched + 2s 重发),IK 精查完再整体刷新。
        self._reachable_markers = self._build_reachable_markers(voxels)
        self._publish()
        self.get_logger().info(
            f"MC 粗扫完成(体素={len(voxels)}),已发布到 RViz;开始 IK 精查(约几分钟,进度见日志)…"
        )

        ik_stats = None
        if self.grid_ik_enabled:
            ik_stats = self._grid_ik_fill(voxels, passers, cos_tol)

        self._log_summary(voxels, n_ori, n_manip, n_free, ik_stats)
        return self._build_reachable_markers(voxels)

    def _grid_ik_fill(self, voxels: dict, passers: list, cos_tol: float):
        """前方 ROI 逐体素 IK 精查:MC 采不到的薄流形区(前下方折叠构型)靠定向 IK 给真值。
        对每个空体素中心,依次用 [上一个成功解(扫描线 warm start), 最近的 MC 种子, 关节中位]
        做初值、对多个 yaw 变体的竖直朝下目标跑 DLS-IK;任一解满足 位置≤半体素 + 轴锥 + 无奇异
        + 无碰撞 即记为可达。全部失败 → 保持空(在多种子下仍失败,才有"大概率不可达"的意义)。"""
        xmin, xmax, ymin, ymax, zmin, zmax = self.grid_ik_region
        vs = self.voxel_size
        # yaw 变体:Rz(ψ)@VERTICAL_DOWN 保持 x 轴朝 -Z、绕竖直轴转 ψ,覆盖不同腕部取向分支。
        yaw_targets = []
        for k in range(max(1, self.grid_ik_yaw_count)):
            psi = 2.0 * math.pi * k / max(1, self.grid_ik_yaw_count)
            c, s = math.cos(psi), math.sin(psi)
            rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            yaw_targets.append(rz @ VERTICAL_DOWN)
        seed_positions = np.array([p for p, _ in passers]) if passers else np.zeros((0, 3))
        q_mid = 0.5 * (self.ik.q_lower + self.ik.q_upper)
        r_ee = float(self.sample_radii[-1]) if len(self.sample_radii) else 0.02
        # 臂展预筛:臂基座(arm_base_Link,固定帧)到体素中心超出可达半径的直接跳过。
        arm_base_id = self.mpc.model.getFrameId("arm_base_Link")
        pin.framesForwardKinematics(self.mpc.model, self.mpc.data, q_mid)
        arm_base_pos = self.mpc.data.oMf[arm_base_id].translation.copy()
        reach_max = 0.60  # 略大于实测可达半径(MC bbox ≈ 0.52),留余量

        keys = []
        for kx in range(int(math.floor(xmin / vs)), int(math.ceil(xmax / vs))):
            for ky in range(int(math.floor(ymin / vs)), int(math.ceil(ymax / vs))):
                for kz in range(int(math.floor(zmin / vs)), int(math.ceil(zmax / vs))):
                    if (kx, ky, kz) not in voxels:
                        keys.append((kx, ky, kz))
        # 扫描线顺序(x→y→z 已按嵌套序)让相邻体素可用上一个解 warm start。
        n_try = n_fill = n_skip = 0
        last_q = None
        log_every = max(1, len(keys) // 5)
        for idx, key in enumerate(keys):
            if idx % log_every == 0 and idx > 0:
                self.get_logger().info(
                    f"  IK 精查进度 {100*idx//len(keys)}%  已试={n_try} 新增可达={n_fill} 预筛跳过={n_skip}"
                )
                # 边算边刷新,RViz 能看到区域逐步被填上。
                self._reachable_markers = self._build_reachable_markers(voxels)
                self._publish()
            center = np.array([(key[0] + 0.5) * vs, (key[1] + 0.5) * vs, (key[2] + 0.5) * vs])
            # 预筛 1:超出臂展 → 必不可达。
            if float(np.linalg.norm(center - arm_base_pos)) > reach_max:
                n_skip += 1
                continue
            # 预筛 2:末端点本身已在障碍膨胀区内 → 必然违规,不用跑 IK。
            blocked = any(
                _disk_signed_distance_np(center, d) - r_ee - self.obstacle_safety_margin < self.extra_clearance
                for d in self.disks
            ) or any(
                _box_signed_distance_np(center, b) - r_ee - self.body_obstacle_margin < self.extra_clearance
                for b in self.boxes
            )
            if blocked:
                n_skip += 1
                continue
            n_try += 1
            seeds = []
            if last_q is not None:
                seeds.append(last_q)
            if len(seed_positions):
                nearest = int(np.argmin(np.linalg.norm(seed_positions - center, axis=1)))
                seeds.append(passers[nearest][1])
            seeds.append(q_mid)
            found = False
            for q0 in seeds:
                for R_target in yaw_targets:
                    q_sol, _ = self.ik.solve(
                        center, target_rotation=R_target, q_init=q0,
                        max_iter=120, eps=5e-4, rot_weight=0.3,
                    )
                    pos, R = self.mpc.forward(q_sol)
                    if float(np.linalg.norm(pos - center)) > 0.5 * vs:
                        continue
                    if self.require_vertical_down and not self._orientation_ok(R, cos_tol):
                        continue
                    manip = self._manipulability(q_sol)
                    if manip < self.manip_threshold:
                        continue
                    if not self._collision_free(self.mpc.capsule_sample_positions(q_sol)):
                        continue
                    voxels[key] = max(voxels.get(key, 0.0), manip)
                    last_q = q_sol
                    n_fill += 1
                    found = True
                    break
                if found:
                    break
        return {"tried": n_try, "filled": n_fill, "skipped": n_skip, "total": len(keys)}

    def _log_summary(self, voxels, n_ori, n_manip, n_free, ik_stats=None):
        if not voxels:
            self.get_logger().warn(
                "没有任何构型通过筛选!放宽条件试试:调低 manipulability_threshold / 调大 orientation_tol_deg / "
                "require_vertical_down:=false / 增大 num_samples。"
            )
            return
        centers = np.array([[(k[0] + 0.5) * self.voxel_size,
                             (k[1] + 0.5) * self.voxel_size,
                             (k[2] + 0.5) * self.voxel_size] for k in voxels])
        manips = np.array(list(voxels.values()))
        lo = centers.min(0)
        hi = centers.max(0)
        ik_line = ""
        if ik_stats:
            ik_line = (
                f"\n  IK 精查: ROI 空体素={ik_stats['total']} 预筛跳过(障碍内)={ik_stats['skipped']} "
                f"尝试={ik_stats['tried']} 新增可达={ik_stats['filled']}"
            )
        self.get_logger().info(
            f"探测完成: 采样={self.num_samples} 过姿态={n_ori} 过奇异={n_manip} 过碰撞(可达)={n_free} "
            f"体素={len(voxels)}{ik_line}\n"
            f"  可达 bbox: x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] (m)\n"
            f"  可操作度: min={manips.min():.4f} max={manips.max():.4f} (阈值 {self.manip_threshold:.4f})"
        )

    # ---- Marker ----
    @staticmethod
    def _manip_color(manip: float, mn: float, mx: float) -> ColorRGBA:
        # 归一化到 [0,1]:0=接近奇异阈值(红) → 0.5(黄) → 1=最远离奇异(绿)。
        t = (manip - mn) / (mx - mn) if mx > mn else 1.0
        t = min(1.0, max(0.0, t))
        return ColorRGBA(r=float(min(1.0, 2.0 * (1.0 - t))), g=float(min(1.0, 2.0 * t)), b=0.0, a=0.9)

    def _build_reachable_markers(self, voxels: dict) -> MarkerArray:
        markers = MarkerArray()
        if not voxels:
            return markers
        manips = list(voxels.values())
        mn, mx = min(manips), max(manips)
        cube = Marker()
        cube.header.frame_id = BASE_FRAME
        cube.ns = "reachable_workspace"
        cube.id = 0
        cube.type = Marker.CUBE_LIST
        cube.action = Marker.ADD
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = float(self.voxel_size)
        for key, manip in voxels.items():
            cube.points.append(Point(
                x=float((key[0] + 0.5) * self.voxel_size),
                y=float((key[1] + 0.5) * self.voxel_size),
                z=float((key[2] + 0.5) * self.voxel_size),
            ))
            cube.colors.append(self._manip_color(manip, mn, mx))
        markers.markers.append(cube)
        return markers

    def _build_obstacle_markers(self) -> MarkerArray:
        markers = MarkerArray()
        for i, disk in enumerate(self.disks):
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.ns = "front_propeller_disks"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(disk[0])
            m.pose.position.y = float(disk[1])
            m.pose.position.z = float(disk[2])
            qx, qy, qz, qw = _quat_from_z_axis(disk[3:6])
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
            diameter = 2.0 * float(disk[6] + self.obstacle_safety_margin)
            m.scale.x = m.scale.y = diameter
            m.scale.z = 0.04
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.15, 0.05, 0.25
            markers.markers.append(m)
        for i, box in enumerate(self.boxes):
            m = Marker()
            m.header.frame_id = BASE_FRAME
            m.ns = "body_obstacle_boxes"
            m.id = 500 + i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(box[0])
            m.pose.position.y = float(box[1])
            m.pose.position.z = float(box[2])
            m.pose.orientation.w = 1.0
            half = box[3:] + self.body_obstacle_margin
            m.scale.x = float(2.0 * half[0])
            m.scale.y = float(2.0 * half[1])
            m.scale.z = float(2.0 * half[2])
            m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.15, 1.0, 0.20
            markers.markers.append(m)
        return markers

    def _publish(self):
        now = self.get_clock().now().to_msg()
        for arr in (self._reachable_markers, self._obstacle_markers):
            for m in arr.markers:
                m.header.stamp = now
            if arr.markers:
                (self.reachable_pub if arr is self._reachable_markers else self.obstacle_pub).publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = LL100WorkspaceProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
