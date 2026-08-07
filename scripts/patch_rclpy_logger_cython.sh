#!/bin/bash
# PATCH rclpy logger for Cython-compiled modules (RDK X5 / ROS2 Humble)
#
# 背景 (2026-08-07): tracked_vehicle 部署产物用 Cython 编译为 .so 后,
# perception_node 启动时在 logger.warn() 崩溃:
#   ValueError: Logger severity cannot be changed between calls.
#
# 根因: rclpy (rcutils_logger.py) 按 CallerId (文件名+行号+函数名) 缓存日志上下文,
# 同一 caller 不允许改变 severity. 明文 .py 时帧信息精确; Cython .so 的
# inspect 帧信息退化, 多个源码位置的 info/warn 调用被识别为同一 caller → 报错.
#
# 修复: 将 "severity 改变报错" 改为 "更新上下文" (日志功能不受影响).
# 幂等: 重复执行安全 (模式不存在则跳过).
set -e
F=/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/impl/rcutils_logger.py
[ -f "$F" ] || { echo "ERROR: $F not found"; exit 1; }
if ! grep -q 'PATCH (2026-08-07)' "$F"; then
  cp "$F" "$F.bak"
  python3 - "$F" << 'PYEOF'
import sys
p = sys.argv[1]
src = open(p).read()
old = """            # Don't support any changes to the logger.
            if severity != context['severity']:
                raise ValueError('Logger severity cannot be changed between calls.')"""
new = """            # Don't support any changes to the logger.
            # PATCH (2026-08-07): Cython 编译模块帧信息不可靠, 同一 caller 可能被
            # 不同 severity 调用 (明文 .py 正常, .so 后触发 ValueError 崩溃).
            # 改为更新上下文而非报错 — 日志功能不受影响.
            if severity != context['severity']:
                context['severity'] = severity"""
assert old in src, 'pattern not found'
open(p, 'w').write(src.replace(old, new))
PYEOF
  echo "PATCHED: $F"
else
  echo "ALREADY PATCHED: $F"
fi
