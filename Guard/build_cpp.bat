@echo off
REM IMU Filter C++ Extension Build Script for Windows
REM 用法：双击运行或在命令行执行 build_cpp.bat

echo ========================================
echo IMU Filter C++ Extension Build Script
echo ========================================
echo.

REM 检查 Python 是否安装
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 3.6+
    pause
    exit /b 1
)

echo [1/4] 检查依赖...
pip show pybind11 >nul 2>&1
if errorlevel 1 (
    echo [安装] 正在安装 pybind11...
    pip install pybind11
    if errorlevel 1 (
        echo [错误] pybind11 安装失败
        pause
        exit /b 1
    )
) else (
    echo [OK] pybind11 已安装
)

echo.
echo [2/4] 清理旧的构建文件...
if exist build rmdir /s /q build
if exist *.pyd del /q *.pyd

echo.
echo [3/4] 编译 C++ 扩展模块...
python setup.py build_ext --inplace
if errorlevel 1 (
    echo.
    echo [错误] 编译失败！
    echo.
    echo 可能的解决方案：
    echo 1. 安装 Visual Studio Build Tools
    echo    https://visualstudio.microsoft.com/visual-cpp-build-tools/
    echo 2. 或使用纯 Python 版本（无需编译）
    echo    python imu_monitor.py
    echo.
    pause
    exit /b 1
)

echo.
echo [4/4] 检查编译结果...
dir /b *.pyd >nul 2>&1
if errorlevel 1 (
    echo [警告] 未找到编译后的 .pyd 文件
) else (
    echo [成功] C++ 扩展模块编译完成！
    dir /b *.pyd
)

echo.
echo ========================================
echo 编译完成！
echo.
echo 运行监视器：
echo   python imu_monitor.py
echo ========================================
echo.
pause
