#!/usr/bin/env python3
"""编译 GRAMPC LL100 C++ 问题描述模块(ll100_grampc_problem.*.so)。

流程:
  1. 调用 grampc_codegen.generate() 确保 CasADi C 代码 + 元信息已生成。
  2. 用 scikit-build-core 的 CMaker(与 pygrampc 官方 examples/Crane2D 同款)配置 + 编译。
     通过 CMake -DPYGRAMPC_ROOT=<scratchpad/pygrampc> 提供 GRAMPC + ProblemDescription 头。
  3. 把编译出的 .so 拷到 grampc_problem/ 根,供 grampc_ll100.py 直接 import。

可被 grampc_ll100.py 在首次运行时自动调用(检测到 .so 缺失/过期时)。
"""

import glob
import importlib
import os
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
GEN_DIR = HERE / "grampc_problem"
MODULE_NAME = "ll100_grampc_problem"
GEN_BASENAME = "ll100_grampc_gen"
META_NAME = "ll100_grampc_meta.json"

# Files that define the generated GRAMPC constraints/costs.  If any of these is
# newer than the generated problem module, rebuild so GRAMPC cannot silently use
# an older collision geometry than the acados backend.
CODEGEN_DEPS = (
    HERE / "mpc_pinocchio_ik.py",
    HERE / "grampc_codegen.py",
    HERE / "benchmark_nmpc_solvers.py",
    HERE / "move_ll100_lh420_nmpc_demo.py",
)
GENERATED_FILES = (
    GEN_DIR / f"{GEN_BASENAME}.c",
    GEN_DIR / f"{GEN_BASENAME}.h",
    GEN_DIR / META_NAME,
)
CXX_BUILD_DEPS = (
    GEN_DIR / "ll100_problem.cpp",
    GEN_DIR / "CMakeLists.txt",
)

# pygrampc 源码树(含 src/ 与 grampc/include),由任务环境放在 scratchpad。
_SCRATCH = (
    "/tmp/claude-1000/-home-windylab-code-windylab-ws/"
    "6ca90882-24d2-46a6-9424-d3ac0bed62e2/scratchpad"
)
PYGRAMPC_ROOT = Path(os.environ.get("PYGRAMPC_ROOT", f"{_SCRATCH}/pygrampc"))


def _pybind11_cmake_dir() -> str | None:
    """返回与已安装 pygrampc._core 内部 ABI 匹配的 pybind11 cmake 目录。

    关键坑:ll100_grampc_problem 必须用与 pygrampc._core 相同的 pybind11 ABI。
    pygrampc 的 pyproject.toml 钉住 pybind11==2.12.0(内部 registry v4);若本机
    find_package 选到 3.x(v11),跨模块继承 grampc::ProblemDescription 会失败,
    import 报 "unknown base type" 或 "ProblemDescription is not registered"。
    默认使用当前 Python 环境里的 pybind11.get_cmake_dir();部署脚本会安装 2.12.0。
    """
    env = os.environ.get("PYBIND11_CMAKE_DIR")
    if env:
        return env
    try:
        import pybind11
        return pybind11.get_cmake_dir()
    except Exception:
        return None


def _find_so() -> Path | None:
    hits = sorted(GEN_DIR.glob(f"{MODULE_NAME}*.so"))
    return hits[0] if hits else None


def _path_mtime_ns(path: Path) -> int | None:
    try:
        return int(path.stat().st_mtime_ns)
    except OSError:
        return None


def _newer_than_any(output: Path, deps) -> bool:
    output_mtime = _path_mtime_ns(output)
    if output_mtime is None:
        return True
    for dep in deps:
        dep_mtime = _path_mtime_ns(dep)
        if dep_mtime is not None and dep_mtime > output_mtime:
            return True
    return False


def _codegen_outdated() -> bool:
    return any(_newer_than_any(path, CODEGEN_DEPS) for path in GENERATED_FILES)


def _module_outdated(module_path: Path | None) -> bool:
    if module_path is None:
        return True
    return _codegen_outdated() or _newer_than_any(module_path, (*GENERATED_FILES, *CXX_BUILD_DEPS))


def is_built() -> bool:
    module = _find_so()
    return module is not None and not _module_outdated(module)


def build(force: bool = False) -> Path:
    """编译问题模块,返回 .so 路径。已编译且未过期且非 force 时直接返回。"""
    existing = _find_so()
    if existing is not None and not force and not _module_outdated(existing):
        return existing

    from grampc_codegen import generate

    generate(force=force or _codegen_outdated())

    if not (PYGRAMPC_ROOT / "src" / "problem_description.hpp").exists():
        raise FileNotFoundError(
            f"找不到 pygrampc 源码树:{PYGRAMPC_ROOT}(设 PYGRAMPC_ROOT 环境变量)"
        )

    import scikit_build_core.cmake as skbuild

    build_dir = GEN_DIR / "build"
    cmaker = skbuild.CMaker(
        skbuild.CMake.default_search(),
        GEN_DIR,
        build_dir,
        "Release",
    )
    cache = {"PYGRAMPC_ROOT": str(PYGRAMPC_ROOT)}
    pb_dir = _pybind11_cmake_dir()
    if pb_dir:
        cache["PYBIND11_CMAKE_DIR"] = pb_dir
    cmaker.init_cache(cache)
    cmaker.configure()
    cmaker.build()

    # 把生成的 .so 收集到 GEN_DIR 根。
    built = []
    for pattern in (f"{MODULE_NAME}*.so",):
        built += glob.glob(str(build_dir / "**" / pattern), recursive=True)
        built += glob.glob(str(GEN_DIR / pattern))
    if not built:
        raise RuntimeError("编译完成但没找到 .so 产物")
    src = Path(sorted(built)[0])
    dst = GEN_DIR / src.name
    if src.resolve() != dst.resolve():
        shutil.copy2(src, dst)
    return dst


def ensure_module():
    """保证模块可 import,返回已导入的模块对象。"""
    # 必须先 import pygrampc:它通过 _core 注册 grampc::ProblemDescription 基类型,
    # 我们的模块继承自它,否则 pybind11 报 "unknown base type"。
    import pygrampc  # noqa: F401
    build()
    if str(GEN_DIR) not in sys.path:
        sys.path.insert(0, str(GEN_DIR))
    try:
        return importlib.import_module(MODULE_NAME)
    except ImportError:
        build()
        importlib.invalidate_caches()
        return importlib.import_module(MODULE_NAME)


if __name__ == "__main__":
    force = "--force" in sys.argv
    path = build(force=force)
    print("编译完成:", path)
    mod = ensure_module()
    print("import 成功:", mod.LL100Problem)
