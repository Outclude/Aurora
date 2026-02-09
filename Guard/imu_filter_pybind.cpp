/**
 * @file imu_filter_pybind.cpp
 * @brief 使用 pybind11 将 C++ IMUFilter 类绑定到 Python
 *
 * 编译方法：
 * 1. 安装 pybind11: pip install pybind11
 * 2. 编译: python setup.py build_ext --inplace
 * 3. 或使用: c++ -O3 -Wall -shared -std=c++11 -fPIC \
 *             $(python3 -m pybind11 --includes) \
 *             imu_filter_pybind.cpp imu_filter.cpp -o imu_filter_pybind$(python3-config --extension-suffix)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "imu_filter.h"

namespace py = pybind11;

/**
 * Python 绑定模块
 */
PYBIND11_MODULE(imu_filter_cpp, m) {
    m.doc() = "IMU Filter C++ Implementation (Python Binding)";

    // ==================== 枚举类型 ====================
    py::enum_<FilterType>(m, "FilterType")
        .value("NONE", FILTER_NONE)
        .value("KALMAN", FILTER_KALMAN)
        .value("LOW_PASS", FILTER_LOW_PASS)
        .value("MOVING_AVG", FILTER_MOVING_AVG)
        .export_values();

    // ==================== 数据结构 ====================

    // IMURawData 结构体
    py::class_<IMURawData>(m, "IMURawData")
        .def(py::init<>())
        .def_readwrite("ax", &IMURawData::ax)
        .def_readwrite("ay", &IMURawData::ay)
        .def_readwrite("az", &IMURawData::az)
        .def_readwrite("gx", &IMURawData::gx)
        .def_readwrite("gy", &IMURawData::gy)
        .def_readwrite("gz", &IMURawData::gz);

    // IMUFilteredData 结构体
    py::class_<IMUFilteredData>(m, "IMUFilteredData")
        .def(py::init<>())
        .def_readwrite("ax", &IMUFilteredData::ax)
        .def_readwrite("ay", &IMUFilteredData::ay)
        .def_readwrite("az", &IMUFilteredData::az)
        .def_readwrite("gx", &IMUFilteredData::gx)
        .def_readwrite("gy", &IMUFilteredData::gy)
        .def_readwrite("gz", &IMUFilteredData::gz)
        .def_readwrite("acc_magnitude", &IMUFilteredData::acc_magnitude)
        .def_readwrite("acc_horizontal", &IMUFilteredData::acc_horizontal);

    // ==================== IMUFilter 类 ====================

    py::class_<IMUFilter>(m, "IMUFilter")
        .def(py::init<FilterType>(), py::arg("type") = FILTER_KALMAN)

        // 滤波处理函数
        .def("filter", [](IMUFilter& self, py::dict raw_dict) {
            // 从 Python 字典转换为 IMURawData
            IMURawData raw;
            raw.ax = raw_dict["ax"].cast<float>();
            raw.ay = raw_dict["ay"].cast<float>();
            raw.az = raw_dict["az"].cast<float>();
            raw.gx = raw_dict["gx"].cast<float>();
            raw.gy = raw_dict["gy"].cast<float>();
            raw.gz = raw_dict["gz"].cast<float>();

            // 调用 C++ 滤波函数
            IMUFilteredData filtered = self.filter(raw);

            // 转换为 Python 字典返回
            py::dict result;
            result["ax"] = filtered.ax;
            result["ay"] = filtered.ay;
            result["az"] = filtered.az;
            result["gx"] = filtered.gx;
            result["gy"] = filtered.gy;
            result["gz"] = filtered.gz;
            result["acc_magnitude"] = filtered.acc_magnitude;
            result["acc_horizontal"] = filtered.acc_horizontal;

            return result;
        }, py::arg("raw_data"))

        // 重置函数
        .def("reset", &IMUFilter::reset)

        // 设置滤波器类型
        .def("setFilterType", &IMUFilter::setFilterType, py::arg("type"))

        // 获取当前滤波后的数据
        .def("getCurrentFiltered", [](const IMUFilter& self) {
            IMUFilteredData filtered = self.getCurrentFiltered();
            py::dict result;
            result["ax"] = filtered.ax;
            result["ay"] = filtered.ay;
            result["az"] = filtered.az;
            result["gx"] = filtered.gx;
            result["gy"] = filtered.gy;
            result["gz"] = filtered.gz;
            result["acc_magnitude"] = filtered.acc_magnitude;
            result["acc_horizontal"] = filtered.acc_horizontal;
            return result;
        });

    m.attr("__version__") = "1.0.0";
}
