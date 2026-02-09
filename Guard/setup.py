"""
setup.py - 编译 pybind11 扩展模块

编译方法：
python setup.py build_ext --inplace
"""

from pybind11.setup_helpers import Pybind11Extension, build_ext
from pybind11 import get_cmake_dir
import pybind11

__all__ = ["build_ext"]

# 扩展模块定义
ext_modules = [
    Pybind11Extension(
        "imu_filter_cpp",  # 模块名称
        sources=[
            "imu_filter_pybind.cpp",  # pybind11 绑定代码
            "imu_filter.cpp"          # C++ 滤波器实现
        ],
        extra_compile_args=["-O3"],  # 优化选项
        cxx_std=11,                  # C++11 标准
    ),
]


def setup():
    from setuptools import setup

    setup(
        name="imu_filter_cpp",
        version="1.0.0",
        author="Your Name",
        description="IMU Filter C++ Implementation with Python Binding",
        long_description="",
        ext_modules=ext_modules,
        install_requires=["pybind11"],
        cmdclass={"build_ext": build_ext},
        zip_safe=False,
        python_requires=">=3.6",
    )


if __name__ == "__main__":
    setup()
