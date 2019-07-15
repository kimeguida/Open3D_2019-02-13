// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d_core.h"
#include "open3d_core_trampoline.h"

#include <Core/Camera/PinholeCameraIntrinsic.h>
#include <Core/Camera/PinholeCameraTrajectory.h>
#include <IO/ClassIO/IJsonConvertibleIO.h>
#include <IO/ClassIO/PinholeCameraTrajectoryIO.h>
using namespace open3d;

void pybind_camera(py::module &m) {
    py::class_<PinholeCameraIntrinsic> pinhole_intr(m, "PinholeCameraIntrinsic",
                                                    "PinholeCameraIntrinsic");
    py::detail::bind_default_constructor<PinholeCameraIntrinsic>(pinhole_intr);
    py::detail::bind_copy_functions<PinholeCameraIntrinsic>(pinhole_intr);
    pinhole_intr
            .def(py::init([](int w, int h, double fx, double fy, double cx,
                             double cy) {
                     return new PinholeCameraIntrinsic(w, h, fx, fy, cx, cy);
                 }),
                 "width"_a, "height"_a, "fx"_a, "fy"_a, "cx"_a, "cy"_a)
            .def(py::init([](PinholeCameraIntrinsicParameters param) {
                     return new PinholeCameraIntrinsic(param);
                 }),
                 "param"_a)
            .def("set_intrinsics", &PinholeCameraIntrinsic::SetIntrinsics,
                 "width"_a, "height"_a, "fx"_a, "fy"_a, "cx"_a, "cy"_a)
            .def("get_focal_length", &PinholeCameraIntrinsic::GetFocalLength)
            .def("get_principal_point",
                 &PinholeCameraIntrinsic::GetPrincipalPoint)
            .def("get_skew", &PinholeCameraIntrinsic::GetSkew)
            .def("is_valid", &PinholeCameraIntrinsic::IsValid)
            .def_readwrite("width", &PinholeCameraIntrinsic::width_)
            .def_readwrite("height", &PinholeCameraIntrinsic::height_)
            .def_readwrite("intrinsic_matrix",
                           &PinholeCameraIntrinsic::intrinsic_matrix_)
            .def("__repr__", [](const PinholeCameraIntrinsic &c) {
                return std::string("PinholeCameraIntrinsic with width = ") +
                       std::to_string(c.width_) +
                       std::string(" and height = ") +
                       std::to_string(c.height_) +
                       std::string(
                               ".\nAccess intrinsics with intrinsic_matrix.");
            });
    py::enum_<PinholeCameraIntrinsicParameters>(
            m, "PinholeCameraIntrinsicParameters", py::arithmetic(),
            "PinholeCameraIntrinsicParameters")
            .value("PrimeSenseDefault",
                   PinholeCameraIntrinsicParameters::PrimeSenseDefault)
            .value("Kinect2DepthCameraDefault",
                   PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault)
            .value("Kinect2ColorCameraDefault",
                   PinholeCameraIntrinsicParameters::Kinect2ColorCameraDefault)
            .export_values();

    py::class_<PinholeCameraParameters> pinhole_param(
            m, "PinholeCameraParameters", "PinholeCameraParameters");
    py::detail::bind_default_constructor<PinholeCameraParameters>(
            pinhole_param);
    py::detail::bind_copy_functions<PinholeCameraParameters>(pinhole_param);
    pinhole_param
            .def_readwrite("intrinsic", &PinholeCameraParameters::intrinsic_)
            .def_readwrite("extrinsic", &PinholeCameraParameters::extrinsic_)
            .def("__repr__", [](const PinholeCameraParameters &c) {
                return std::string("PinholeCameraParameters class.\n") +
                       std::string(
                               "Access its data via intrinsic and extrinsic.");
            });

    py::class_<PinholeCameraTrajectory> pinhole_traj(
            m, "PinholeCameraTrajectory", "PinholeCameraTrajectory");
    py::detail::bind_default_constructor<PinholeCameraTrajectory>(pinhole_traj);
    py::detail::bind_copy_functions<PinholeCameraTrajectory>(pinhole_traj);
    pinhole_traj
            .def_readwrite("parameters", &PinholeCameraTrajectory::parameters_)
            .def("__repr__", [](const PinholeCameraTrajectory &c) {
                return std::string("PinholeCameraTrajectory class.\n") +
                       std::string("Access its data via camera_parameters.");
            });
}

void pybind_camera_methods(py::module &m) {
    m.def("read_pinhole_camera_intrinsic",
          [](const std::string &filename) {
              PinholeCameraIntrinsic intrinsic;
              ReadIJsonConvertible(filename, intrinsic);
              return intrinsic;
          },
          "Function to read PinholeCameraIntrinsic from file", "filename"_a);
    m.def("write_pinhole_camera_intrinsic",
          [](const std::string &filename,
             const PinholeCameraIntrinsic &intrinsic) {
              return WriteIJsonConvertible(filename, intrinsic);
          },
          "Function to write PinholeCameraIntrinsic to file", "filename"_a,
          "intrinsic"_a);
    m.def("read_pinhole_camera_parameters",
          [](const std::string &filename) {
              PinholeCameraParameters parameters;
              ReadIJsonConvertible(filename, parameters);
              return parameters;
          },
          "Function to read PinholeCameraParameters from file", "filename"_a);
    m.def("write_pinhole_camera_parameters",
          [](const std::string &filename,
             const PinholeCameraParameters &parameters) {
              return WriteIJsonConvertible(filename, parameters);
          },
          "Function to write PinholeCameraParameters to file", "filename"_a,
          "parameters"_a);
    m.def("read_pinhole_camera_trajectory",
          [](const std::string &filename) {
              PinholeCameraTrajectory trajectory;
              ReadPinholeCameraTrajectory(filename, trajectory);
              return trajectory;
          },
          "Function to read PinholeCameraTrajectory from file", "filename"_a);
    m.def("write_pinhole_camera_trajectory",
          [](const std::string &filename,
             const PinholeCameraTrajectory &trajectory) {
              return WritePinholeCameraTrajectory(filename, trajectory);
          },
          "Function to write PinholeCameraTrajectory to file", "filename"_a,
          "trajectory"_a);
}
