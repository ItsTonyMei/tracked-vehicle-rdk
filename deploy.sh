#!/bin/bash
# tracked_vehicle 重建脚本 — 源码在 Windows 管理, 板端只保留构建产物(部署后自动清理源码)
# 部署产物用 Cython 编译为 .so 机器码 — 板端不落明文源码 (知识产权保护)
set -e
cd /home/sunrise/Desktop/tracked-vehicle-rdk
if [ ! -d src/tracked_vehicle ]; then
  echo 'ERROR: src/tracked_vehicle 不存在 — 请先 scp 源码:'
  echo '  scp -r src/tracked_vehicle root@192.168.0.104:/home/sunrise/Desktop/tracked-vehicle-rdk/src/'
  exit 1
fi
echo '==> 构建 (copy-install 模式)'
source /opt/tros/humble/setup.bash
colcon build --packages-select tracked_vehicle
echo '==> 同步 launch'
mkdir -p install/tracked_vehicle/share/tracked_vehicle
cp -r launch install/tracked_vehicle/share/tracked_vehicle/
echo '==> Cython 编译 (核心模块 → .so 机器码, 板端无明文源码)'
PKG=install/tracked_vehicle/lib/python3.10/site-packages/tracked_vehicle
cd "$PKG"
cythonize -i -3 *.py
rm -f *.py *.pyc *.c        # 删除明文 + 字节码 + C 中间产物
rm -rf __pycache__
cd /home/sunrise/Desktop/tracked-vehicle-rdk
echo '==> 重启服务'
systemctl restart tracked-vehicle-display
sleep 8
N=$(systemctl show tracked-vehicle-display -p NRestarts | cut -d= -f2)
if [ "$N" -lt 2 ]; then
  echo "==> OK: 服务正常 (NRestarts=$N), 清理源码"
  rm -rf src build log
else
  echo "==> WARN: 服务仍重启 ($N) — 保留源码, 检查 journalctl -u tracked-vehicle-display"
fi
