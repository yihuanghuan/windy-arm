#!/usr/bin/env python3
"""LL100-LH420 避障可达范围探测工具。

在关节空间蒙特卡洛采样,筛出"末端竖直朝下 + 无奇异 + 与桨盘/机身无碰撞"的构型,
把通过的末端位置体素化,按可操作度上色发布成 RViz CUBE_LIST,直观看出机械臂在
避障约束下的"安全可达区"。

碰撞模型与主 demo 完全一致(复用同一套 capsule_points / 障碍几何 / margin),
所以吃到用户在 move_ll100_lh420_nmpc_demo.py 里微调过的小球位置。

手动启动(先起控制版 launch 提供 RViz + robot_state_publisher):
    python3 ll100_workspace_probe.py --ros-args -p num_samples:=300000
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

        # 竖直朝下(全姿态匹配)命中率低(~0.1%),默认 1e6 采样(~10s)给出约上千体素;想更密就调大。
        self.num_samples = int(self.declare_parameter("num_samples", 1000000).value)
        self.voxel_size = float(self.declare_parameter("voxel_size", 0.02).value)
        self.manip_threshold = float(self.declare_parameter("manipulability_threshold", 0.012).value)
        self.orientation_tol_deg = float(self.declare_parameter("orientation_tol_deg", 20.0).value)
        self.require_vertical_down = bool(self.declare_parameter("require_vertical_down", True).value)
        self.extra_clearance = float(self.declare_parameter("extra_clearance", 0.0).value)
        self.seed = int(self.declare_parameter("seed", 0).value)
        # 碰撞 margin 默认与主 demo 一致;单独暴露以便按需收紧/放松。
        self.obstacle_safety_margin = float(self.declare_parameter("obstacle_safety_margin", 0.005).value)
        self.body_obstacle_margin = float(self.declare_parameter("body_obstacle_margin", 0.005).value)

        cfg = MpcIkConfig(
            solver_backend="casadi",  # 只用 FK + numpy 碰撞,不 solve/编译 acados
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
        self._reachable_markers = self._compute_and_build()
        self._obstacle_markers = self._build_obstacle_markers()

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

    def _compute_and_build(self) -> MarkerArray:
        mpc = self.mpc
        model, data, fid = mpc.model, mpc.data, mpc.frame_id
        lo, hi = self._joint_bounds()
        rng = np.random.default_rng(self.seed)
        cos_tol = math.cos(math.radians(self.orientation_tol_deg))
        vs = self.voxel_size

        voxels: dict = {}
        n_ori = n_manip = n_free = 0
        log_every = max(1, self.num_samples // 10)

        for i in range(self.num_samples):
            if i % log_every == 0 and i > 0:
                self.get_logger().info(
                    f"  进度 {100*i//self.num_samples}%  过姿态={n_ori} 过奇异={n_manip} 过碰撞={n_free} 体素={len(voxels)}"
                )
            q = rng.uniform(lo, hi)
            pos, R = mpc.forward(q)

            if self.require_vertical_down:
                cos_angle = 0.5 * (float(np.trace(R.T @ VERTICAL_DOWN)) - 1.0)
                if cos_angle < cos_tol:
                    continue
            n_ori += 1

            jac = pin.computeFrameJacobian(model, data, q, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
            manip = math.sqrt(max(float(np.linalg.det(jac @ jac.T)), 0.0))
            if manip < self.manip_threshold:
                continue
            n_manip += 1

            samples = mpc.capsule_sample_positions(q)
            if not self._collision_free(samples):
                continue
            n_free += 1

            key = (
                int(math.floor(pos[0] / vs)),
                int(math.floor(pos[1] / vs)),
                int(math.floor(pos[2] / vs)),
            )
            if key not in voxels or manip > voxels[key]:
                voxels[key] = manip

        self._log_summary(voxels, n_ori, n_manip, n_free)
        return self._build_reachable_markers(voxels)

    def _log_summary(self, voxels, n_ori, n_manip, n_free):
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
        self.get_logger().info(
            f"探测完成: 采样={self.num_samples} 过姿态={n_ori} 过奇异={n_manip} 过碰撞(可达)={n_free} "
            f"体素={len(voxels)}\n"
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
