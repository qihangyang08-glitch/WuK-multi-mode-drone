#!/bin/bash
# WuK SITL Quick Test Script (Fixed)
# 使用默认frame(已劫持为wuk_motors)而不是-f wuk

set -e
cd ~/ardupilot

echo "========================================="
echo "WuK SITL Testing Script"
echo "========================================="
echo ""
echo "[INFO] 使用默认quad frame (已配置为wuk_motors)"
echo "[INFO] 参数文件: Tools/autotest/default_params/wuk.parm"
echo ""
echo "[1/2] Starting SITL..."

# 不使用 -f wuk, 使用默认frame
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w \
    --add-param-file=Tools/autotest/default_params/wuk.parm &

SITL_PID=$!
echo "SITL PID: $SITL_PID"
echo ""
echo "等待SITL启动完成 (约15秒)..."
sleep 15

echo ""
echo "[2/2] SITL已启动"
echo ""
echo "MAVProxy commands:"
echo "  mode LOITER  # 切换到LOITER"
echo "  arm throttle # 解锁"
echo "  rc 3 1600    # 起飞"
echo "  rc 7 1900    # 测试RC7 (UART消息)"
echo "  rc 8 1500    # 测试RC8 (UART消息)"
echo "  rc 9 1900    # 触发变形 (进入MORPH模式)"
echo ""
echo "观察点:"
echo "  1. RC7/8改变时应看到: 'WuK RC7: XXXX -> ACT1'"
echo "  2. RC9触发后应看到: 'Mode MORPH'"
echo "  3. 终端输出: 'SITL WuK: FC angle=X.X° -> PWM=XXXX'"
echo ""
echo "========================================="
echo "按 Ctrl+C 停止SITL"
echo "========================================="
echo ""

wait $SITL_PID
