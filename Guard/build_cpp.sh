#!/bin/bash
# IMU Filter C++ Extension Build Script for Linux/Mac

echo "========================================"
echo "IMU Filter C++ Extension Build Script"
echo "========================================"
echo

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "[错误] 未找到 Python3，请先安装 Python 3.6+"
    exit 1
fi

echo "[1/4] 检查依赖..."
if ! python3 -c "import pybind11" 2>/dev/null; then
    echo "[安装] 正在安装 pybind11..."
    pip3 install pybind11
    if [ $? -ne 0 ]; then
        echo "[错误] pybind11 安装失败"
        exit 1
    fi
else
    echo "[OK] pybind11 已安装"
fi

echo
echo "[2/4] 清理旧的构建文件..."
rm -rf build
rm -f *.so

echo
echo "[3/4] 编译 C++ 扩展模块..."
python3 setup.py build_ext --inplace
if [ $? -ne 0 ]; then
    echo
    echo "[错误] 编译失败！"
    echo
    echo 可能的解决方案：
    echo "1. 安装 Python 开发头文件"
    echo "   Ubuntu/Debian: sudo apt-get install python3-dev"
    echo "   Fedora/RHEL: sudo dnf install python3-devel"
    echo "   Mac: xcode-select --install"
    echo "2. 或使用纯 Python 版本（无需编译）"
    echo "   python3 imu_monitor.py"
    echo
    exit 1
fi

echo
echo "[4/4] 检查编译结果..."
if ls *.so 1> /dev/null 2>&1; then
    echo "[成功] C++ 扩展模块编译完成！"
    ls -lh *.so
else
    echo "[警告] 未找到编译后的 .so 文件"
fi

echo
echo "========================================"
echo "编译完成！"
echo
echo "运行监视器："
echo "  python3 imu_monitor.py"
echo "========================================"
