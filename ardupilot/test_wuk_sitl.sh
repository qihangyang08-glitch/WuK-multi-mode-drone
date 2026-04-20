#!/bin/bash
# WuK SITL Quick Test Script

set -e
cd ~/ardupilot

echo "========================================="
echo "WuK SITL Testing Script"
echo "========================================="
echo ""
echo "[1/2] Starting SITL with WuK frame..."

Tools/autotest/sim_vehicle.py -v ArduCopter -f wuk --console --map \
    --add-param-file=Tools/autotest/default_params/copter-wuk.parm &

SITL_PID=$!
echo "SITL PID: $SITL_PID"
sleep 15

echo ""
echo "[2/2] SITL已启动"
echo "MAVProxy commands:"
echo "  mode LOITER  # 切换到LOITER"
echo "  arm throttle # 解锁"
echo "  rc 3 1600    # 起飞"
echo "  rc 7 1900    # 测试RC7(UART消息)"
echo "  rc 9 1900    # 触发变形"
echo ""
echo "按 Ctrl+C 停止SITL"
echo "========================================="

wait $SITL_PID
