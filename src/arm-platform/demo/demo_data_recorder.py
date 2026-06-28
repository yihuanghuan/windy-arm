#!/usr/bin/env python3
"""机械臂 demo 的数据记录、归档与实时绘图工具。"""

import math
import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np


class DemoDataRecorder:
    def __init__(self,
                 log_root_dir: str,
                 save_data: bool = True,
                 enable_plot: bool = True,
                 realtime_plot: bool = True):
        self.log_root_dir = log_root_dir
        self.save_data = save_data
        self.enable_plot = enable_plot
        self.realtime_plot = realtime_plot and enable_plot

        self.archive_dir = None
        self.log_npy_path = None
        self.log_plot_path = None
        self.log_txt_path = None

        self.logs = {
            't': [],
            'base_pos': [],
            'base_rpy': [],
            'ee_pos': [],
            'ee_rpy': [],
            'ee_target_pos': [],
            'ee_target_rpy': [],
            'ee_pos_err': [],
            'ee_rot_err_rpy': [],
            'ee_pos_err_norm': [],
            'ee_rot_err_norm': [],
        }

        self.fig = None
        self.axes = None

        if self.realtime_plot:
            plt.ion()
            self.fig, self.axes = plt.subplots(5, 2, figsize=(16, 18))
            self.axes = self.axes.reshape(5, 2)
            self.fig.suptitle('Move Base Circle Visual Demo Realtime Plot')
            plt.show(block=False)

    def add_sample(self,
                   t: float,
                   base_pos: np.ndarray,
                   base_rpy: np.ndarray,
                   ee_pos: np.ndarray,
                   ee_rpy: np.ndarray,
                   ee_target_pos: np.ndarray,
                   ee_target_rpy: np.ndarray,
                   ee_pos_err: np.ndarray,
                   ee_rot_err_rpy: np.ndarray):
        self.logs['t'].append(float(t))
        self.logs['base_pos'].append(np.asarray(base_pos, dtype=float).copy())
        self.logs['base_rpy'].append(np.asarray(base_rpy, dtype=float).copy())
        self.logs['ee_pos'].append(np.asarray(ee_pos, dtype=float).copy())
        self.logs['ee_rpy'].append(np.asarray(ee_rpy, dtype=float).copy())
        self.logs['ee_target_pos'].append(np.asarray(ee_target_pos, dtype=float).copy())
        self.logs['ee_target_rpy'].append(np.asarray(ee_target_rpy, dtype=float).copy())
        self.logs['ee_pos_err'].append(np.asarray(ee_pos_err, dtype=float).copy())
        self.logs['ee_rot_err_rpy'].append(np.asarray(ee_rot_err_rpy, dtype=float).copy())
        self.logs['ee_pos_err_norm'].append(float(np.linalg.norm(ee_pos_err)))
        self.logs['ee_rot_err_norm'].append(float(np.linalg.norm(ee_rot_err_rpy)))

        if self.realtime_plot:
            self.update_realtime_plot()

    def has_data(self) -> bool:
        return len(self.logs['t']) > 0

    def to_numpy_dict(self):
        return {key: np.asarray(value) for key, value in self.logs.items()}

    def _prepare_archive_paths(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.archive_dir = os.path.join(self.log_root_dir, timestamp)
        os.makedirs(self.archive_dir, exist_ok=True)
        self.log_npy_path = os.path.join(self.archive_dir, f'{timestamp}_log.npz')
        self.log_plot_path = os.path.join(self.archive_dir, f'{timestamp}_plot.png')
        self.log_txt_path = os.path.join(self.archive_dir, f'{timestamp}_summary.txt')

    def _render_plots(self, data):
        if self.axes is None:
            self.fig, self.axes = plt.subplots(5, 2, figsize=(16, 18))
            self.axes = self.axes.reshape(5, 2)
        else:
            for ax in self.axes.flat:
                ax.clear()

        t = data['t']
        axes = self.axes

        axes[0, 0].plot(t, data['base_pos'][:, 0], label='x')
        axes[0, 0].plot(t, data['base_pos'][:, 1], label='y')
        axes[0, 0].plot(t, data['base_pos'][:, 2], label='z')
        axes[0, 0].set_title('Base Position')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].plot(t, np.rad2deg(data['base_rpy'][:, 0]), label='roll')
        axes[0, 1].plot(t, np.rad2deg(data['base_rpy'][:, 1]), label='pitch')
        axes[0, 1].plot(t, np.rad2deg(data['base_rpy'][:, 2]), label='yaw')
        axes[0, 1].set_title('Base Attitude (deg)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        axes[1, 0].plot(t, data['ee_pos'][:, 0], label='actual x')
        axes[1, 0].plot(t, data['ee_pos'][:, 1], label='actual y')
        axes[1, 0].plot(t, data['ee_pos'][:, 2], label='actual z')
        axes[1, 0].plot(t, data['ee_target_pos'][:, 0], '--', label='target x')
        axes[1, 0].plot(t, data['ee_target_pos'][:, 1], '--', label='target y')
        axes[1, 0].plot(t, data['ee_target_pos'][:, 2], '--', label='target z')
        axes[1, 0].set_title('End-Effector Position')
        axes[1, 0].legend(ncol=2)
        axes[1, 0].grid(True)

        axes[1, 1].plot(t, np.rad2deg(data['ee_rpy'][:, 0]), label='actual roll')
        axes[1, 1].plot(t, np.rad2deg(data['ee_rpy'][:, 1]), label='actual pitch')
        axes[1, 1].plot(t, np.rad2deg(data['ee_rpy'][:, 2]), label='actual yaw')
        axes[1, 1].plot(t, np.rad2deg(data['ee_target_rpy'][:, 0]), '--', label='target roll')
        axes[1, 1].plot(t, np.rad2deg(data['ee_target_rpy'][:, 1]), '--', label='target pitch')
        axes[1, 1].plot(t, np.rad2deg(data['ee_target_rpy'][:, 2]), '--', label='target yaw')
        axes[1, 1].set_title('End-Effector Attitude (deg)')
        axes[1, 1].legend(ncol=2)
        axes[1, 1].grid(True)

        axes[2, 0].plot(t, data['ee_pos_err'][:, 0], label='ex')
        axes[2, 0].plot(t, data['ee_pos_err'][:, 1], label='ey')
        axes[2, 0].plot(t, data['ee_pos_err'][:, 2], label='ez')
        axes[2, 0].set_title('Position Error (m)')
        axes[2, 0].legend()
        axes[2, 0].grid(True)

        axes[2, 1].plot(t, np.rad2deg(data['ee_rot_err_rpy'][:, 0]), label='eroll')
        axes[2, 1].plot(t, np.rad2deg(data['ee_rot_err_rpy'][:, 1]), label='epitch')
        axes[2, 1].plot(t, np.rad2deg(data['ee_rot_err_rpy'][:, 2]), label='eyaw')
        axes[2, 1].set_title('Attitude Error (deg)')
        axes[2, 1].legend()
        axes[2, 1].grid(True)

        axes[3, 0].plot(t, data['ee_pos_err_norm'])
        axes[3, 0].set_title('Position Error Norm (m)')
        axes[3, 0].grid(True)

        axes[3, 1].plot(t, np.rad2deg(data['ee_rot_err_norm']))
        axes[3, 1].set_title('Attitude Error Norm (deg)')
        axes[3, 1].grid(True)

        axes[4, 0].plot(data['ee_target_pos'][:, 1], data['ee_target_pos'][:, 2], '--', label='target yz')
        axes[4, 0].plot(data['ee_pos'][:, 1], data['ee_pos'][:, 2], label='actual yz')
        axes[4, 0].set_title('End-Effector Trajectory in YZ')
        axes[4, 0].set_xlabel('y (m)')
        axes[4, 0].set_ylabel('z (m)')
        axes[4, 0].legend()
        axes[4, 0].grid(True)

        axes[4, 1].plot(data['base_pos'][:, 0], data['base_pos'][:, 1], label='base xy')
        axes[4, 1].set_title('Base Trajectory in XY')
        axes[4, 1].set_xlabel('x (m)')
        axes[4, 1].set_ylabel('y (m)')
        axes[4, 1].legend()
        axes[4, 1].grid(True)

        time_series_axes = [
            axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1],
            axes[2, 0], axes[2, 1], axes[3, 0], axes[3, 1],
        ]
        for ax in time_series_axes:
            if len(t) > 1:
                ax.set_xlim(t[0], t[-1])

        self.fig.tight_layout()

    def update_realtime_plot(self):
        if not self.realtime_plot or not self.has_data():
            return
        data = self.to_numpy_dict()
        self._render_plots(data)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _write_summary_text(self, data, metadata: dict):
        base_pos_min = np.min(data['base_pos'], axis=0)
        base_pos_max = np.max(data['base_pos'], axis=0)
        base_rpy_min_deg = np.rad2deg(np.min(data['base_rpy'], axis=0))
        base_rpy_max_deg = np.rad2deg(np.max(data['base_rpy'], axis=0))
        pos_err_max = np.max(data['ee_pos_err_norm'])
        pos_err_mean = np.mean(data['ee_pos_err_norm'])
        rot_err_max_deg = np.rad2deg(np.max(data['ee_rot_err_norm']))
        rot_err_mean_deg = np.rad2deg(np.mean(data['ee_rot_err_norm']))

        lines = [
            'move_base_circle_visual_demo 运行归档',
            f'归档目录: {self.archive_dir}',
            f'采样点数: {len(data["t"])}',
            f'总时长: {data["t"][-1] - data["t"][0]:.3f} s',
            '',
            '参数:',
        ]
        for key, value in metadata.items():
            lines.append(f'  {key} = {value}')
        lines.extend([
            '',
            '基座位置范围:',
            f'  min = {base_pos_min.tolist()}',
            f'  max = {base_pos_max.tolist()}',
            '基座姿态范围 (deg):',
            f'  min = {base_rpy_min_deg.tolist()}',
            f'  max = {base_rpy_max_deg.tolist()}',
            '',
            '末端误差统计:',
            f'  position_error_norm_mean = {pos_err_mean:.6f} m',
            f'  position_error_norm_max  = {pos_err_max:.6f} m',
            f'  attitude_error_norm_mean = {rot_err_mean_deg:.6f} deg',
            f'  attitude_error_norm_max  = {rot_err_max_deg:.6f} deg',
            '',
            '文件列表:',
            f'  npz: {self.log_npy_path}',
            f'  png: {self.log_plot_path}',
            f'  txt: {self.log_txt_path}',
        ])

        with open(self.log_txt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))

    def finalize(self, metadata: dict):
        if not self.has_data():
            return None

        data = self.to_numpy_dict()
        archive_created = False

        if self.save_data or self.enable_plot:
            self._prepare_archive_paths()
            archive_created = True

        if self.save_data:
            np.savez(self.log_npy_path, **data)
            self._write_summary_text(data, metadata)

        if self.enable_plot:
            self._render_plots(data)
            self.fig.savefig(self.log_plot_path, dpi=150)

        if self.realtime_plot and self.fig is not None:
            plt.ioff()
            plt.show(block=False)

        return {
            'archive_dir': self.archive_dir if archive_created else None,
            'log_npy_path': self.log_npy_path if self.save_data else None,
            'log_plot_path': self.log_plot_path if self.enable_plot else None,
            'log_txt_path': self.log_txt_path if self.save_data else None,
        }
