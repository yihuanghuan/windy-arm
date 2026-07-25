#!/usr/bin/env python3
"""采集实验源码/构建 provenance,写成一个 JSON 供 demo 快照嵌入。

**在控制开始前由实验脚本调用**(此时相关 ROS 节点已起),把逐文件 SHA256、
外层/内层 git 状态、student_arm_node/visualizer 实际参数一次性采集落盘。
这样 demo 控制 timer 内**不做任何 git/hash/param dump 阻塞操作**。

用法:
  python3 collect_provenance.py --out data/provenance_baseline.json \
    [--urdf <path>] [--dump-node /student_arm_node,/trajectory_visualizer_demo]
"""

import argparse
import hashlib
import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

DEMO_DIR = Path(__file__).resolve().parent
PKG_DIR = DEMO_DIR.parent          # src/arm-platform
WS_DIR = PKG_DIR.parent.parent     # workspace root (…/windylab_ws)

# 运行关键文件(相对 PKG_DIR 或绝对);构建产物单列。
KEY_FILES = {
    'demo/move_base_circle_mpc_ik_demo.py': DEMO_DIR / 'move_base_circle_mpc_ik_demo.py',
    'demo/mpc_pinocchio_ik.py': DEMO_DIR / 'mpc_pinocchio_ik.py',
    'demo/pinocchio_ik.py': DEMO_DIR / 'pinocchio_ik.py',
    'demo/moving_base_trajectory_config.py': DEMO_DIR / 'moving_base_trajectory_config.py',
    'demo/trajectory_visualizer_demo.py': DEMO_DIR / 'trajectory_visualizer_demo.py',
    'demo/analyze_moving_base_run.py': DEMO_DIR / 'analyze_moving_base_run.py',
    'launch/student_arm.launch.py': PKG_DIR / 'launch' / 'student_arm.launch.py',
    'config/arm.urdf': PKG_DIR / 'config' / 'arm.urdf',
    'config/motor_config.yaml': PKG_DIR / 'config' / 'motor_config.yaml',
    'config/arm_config.yaml': PKG_DIR / 'config' / 'arm_config.yaml',
    'build/student_arm_node': WS_DIR / 'install' / 'manipulator' / 'lib' / 'manipulator' / 'student_arm_node',
    # 安装 share 下实际被 launch 使用的副本
    'install/arm.urdf': WS_DIR / 'install' / 'manipulator' / 'share' / 'manipulator' / 'arm.urdf',
}


def sha256_file(path: Path):
    try:
        h = hashlib.sha256()
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 16), b''):
                h.update(chunk)
        return {'sha256': h.hexdigest(), 'bytes': path.stat().st_size, 'path': str(path)}
    except Exception as exc:
        return {'sha256': None, 'error': str(exc), 'path': str(path)}


def git_repo_info(start: Path):
    def run(cmd, cwd):
        try:
            return subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True,
                                  timeout=10.0).stdout.strip()
        except Exception:
            return ''
    root = run(['git', 'rev-parse', '--show-toplevel'], start)
    if not root:
        return {'root': None, 'note': 'no git repo'}
    return {
        'root': root,
        'head': run(['git', 'rev-parse', 'HEAD'], start),
        'dirty': bool(run(['git', 'status', '--porcelain'], start)),
        'branch': run(['git', 'rev-parse', '--abbrev-ref', 'HEAD'], start),
    }


# 阶段1D:结构化提取执行器关键参数,使 gain 能成为唯一实验变量,同时仍逐值核对其余六项。
EXECUTOR_KEY_PARAMS = (
    'max_velocity', 'enable_velocity_feedforward', 'velocity_feedforward_gain',
    'position_feedback_tau', 'feedback_velocity_limit',
    'enable_acceleration_limit', 'max_acceleration',
)


def _extract_key_params(yaml_text, node_name):
    """从 ros2 param dump 的 YAML 里结构化取执行器关键参数(找不到记 None,不静默)。"""
    try:
        import yaml
        doc = yaml.safe_load(yaml_text) or {}
    except Exception as exc:
        return {'_parse_error': str(exc)}
    params = ((doc.get(node_name) or {}).get('ros__parameters')
              or (doc.get(node_name.lstrip('/')) or {}).get('ros__parameters') or {})
    return {k: params.get(k, None) for k in EXECUTOR_KEY_PARAMS}


def dump_node_params(node_name):
    try:
        out = subprocess.run(['ros2', 'param', 'dump', node_name, '--print'],
                             capture_output=True, text=True, timeout=20.0)
        text = out.stdout.strip()
        if not text:
            return f'(empty dump rc={out.returncode} err={out.stderr.strip()[:200]})'
        result = {'yaml': text, 'sha256': hashlib.sha256(text.encode()).hexdigest()}
        if node_name.rstrip('/').endswith('student_arm_node'):
            result['key_params'] = _extract_key_params(text, node_name)
        return result
    except Exception as exc:
        return f'(dump failed: {exc})'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--urdf', default=None, help='实际使用的 URDF(覆盖默认 config/arm.urdf 项)')
    p.add_argument('--dump-node', default='/student_arm_node,/trajectory_visualizer_demo')
    p.add_argument('--label', default='')
    args = p.parse_args()

    files = {name: sha256_file(path) for name, path in KEY_FILES.items()}
    if args.urdf:
        files['runtime_urdf'] = sha256_file(Path(args.urdf))

    node_params = {}
    for node in [n for n in args.dump_node.split(',') if n]:
        node_params[node] = dump_node_params(node)

    prov = {
        'schema': 'provenance/v1',
        'label': args.label,
        # created 时间戳仅记录用,不进指纹比较
        'created': datetime.now().isoformat(timespec='seconds'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '(unset=0)'),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', '(default)'),
        'git': {
            'inner_pkg': git_repo_info(PKG_DIR),
            'outer_ws': git_repo_info(WS_DIR),
        },
        'files_sha256': files,
        'node_params': node_params,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(prov, indent=2, ensure_ascii=False))
    missing = [n for n, v in files.items() if v.get('sha256') is None]
    print(f'[provenance] wrote {out}; files={len(files)} missing={missing}')


if __name__ == '__main__':
    main()
