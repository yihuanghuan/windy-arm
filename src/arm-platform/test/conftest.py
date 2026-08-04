"""在任何测试模块导入之前设好布局环境变量。

`task_space_smc.core.trajectory_reference` 依赖的轨迹配置模块在【模块级】读
``MB_STATION_LAYOUT``，读完就固化。因此谁先 import 谁决定布局。

原先 test_pen_axis_convention.py 自己在文件顶部 setdefault 后再 import，
靠的是"它是第一个 import 轨迹模块的测试文件"这个巧合。2026-08-04 新增
test_marker_publish_cost.py（字母序在 pen 之前）导入了
whole_task_smc_ros_runner，轨迹模块就以默认的 vertical（7 关节历史布局）
初始化了，于是笔轴测试报"笔轴与书写平面法向夹角 90°"——
**单独跑通过、全套跑失败**。

这不是运行器的功能缺陷：实际运行时 launch 文件在起节点之前就设好了
（task_space_smc_whole6.launch.py 里的 os.environ.setdefault）。
纯粹是测试顺序脆弱性，放进 conftest 就与顺序无关了。

⚠️ 不要把这里改成无条件赋值 —— 用 setdefault 才能让外部显式指定
其他布局时测试仍按外部意图跑。
"""

import os

# 6 关节实机构型用 horizontal；默认 vertical 是 7 关节的历史布局。
os.environ.setdefault("MB_STATION_LAYOUT", "horizontal")

# 2026-08-04 目录分层后：测试文件下沉到 test/core|sim|hw/，各文件里那句
# sys.path 的相对深度全部要 +1。与其依赖每个文件都改对，在这里统一插一次
# —— conftest 永远在任何测试模块导入之前执行，且它自己固定在 test/ 根，
# parents[1] 就是包的父目录（src/arm-platform），与测试放在第几层无关。
import sys
from pathlib import Path

_PKG_PARENT = str(Path(__file__).resolve().parents[1])
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)
