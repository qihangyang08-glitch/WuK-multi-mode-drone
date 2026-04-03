#!/bin/bash
# WuK SITL 交互式测试脚本
# 启动SITL后保持运行,等待用户交互

set -e
cd ~/ardupilot

echo "========================================="
echo "WuK SITL 交互式测试脚本"
echo "========================================="
echo ""
echo "启动SITL (需要手动交互)"
echo ""

# 启动SITL并保持运行
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map -w \
    --add-param-file=Tools/autotest/default_params/wuk.parm

# SITL关闭后执行后处理
echo ""
echo "========================================="
echo "SITL已关闭,进行后处理..."
echo "========================================="
echo ""

# 检查日志
if [ ! -f "wuk_gb.log" ]; then
    echo "❌ wuk_gb.log 不存在"
    exit 1
fi

echo "✅ 找到日志文件:"
ls -lh wuk_gb.log wuk_gb.jsonl 2>/dev/null || true
echo ""

# 解析日志
if [ -f "tools/wuk_log_parser" ]; then
    echo "[分析] 运行日志解析器..."
    ./tools/wuk_log_parser wuk_gb.log
    echo ""
fi

# 生成图表
if [ -f "tools/wuk_plot.py" ]; then
    echo "[可视化] 生成分析图表..."
    python3 tools/wuk_plot.py
    echo ""
    ls -lh wuk_*.png 2>/dev/null && echo "✅ 图表生成成功" || echo "⚠️  未生成图表"
fi

echo ""
echo "========================================="
echo "测试流程完成!"
echo "========================================="
