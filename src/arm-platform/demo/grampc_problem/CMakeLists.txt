cmake_minimum_required(VERSION 3.15...3.26)
project(ll100_grampc_problem LANGUAGES CXX C)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
# 必须用与已安装 pygrampc._core 相同的 pybind11 ABI(当前部署为 2.12.0),
# 否则 pybind11 内部 type registry 版本号不匹配,导致 import 时
# "referenced unknown base type grampc::ProblemDescription"。
if(DEFINED PYBIND11_CMAKE_DIR)
    set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
endif()
find_package(pybind11 CONFIG REQUIRED)

# PYGRAMPC_ROOT 指向 scratchpad 里的 pygrampc 源码树(含 src/、grampc/include)。
if(NOT DEFINED PYGRAMPC_ROOT)
    message(FATAL_ERROR "PYGRAMPC_ROOT 未定义,请用 -DPYGRAMPC_ROOT=<pygrampc 源码路径> 配置")
endif()

include_directories(
    ${PYGRAMPC_ROOT}/src
    ${PYGRAMPC_ROOT}/grampc/include
    /usr/include/eigen3
)

# CasADi 生成的 C 导数函数 + 我们的问题类一起编进同一个模块。
pybind11_add_module(ll100_grampc_problem MODULE
    ll100_problem.cpp
    ll100_grampc_gen.c
)
