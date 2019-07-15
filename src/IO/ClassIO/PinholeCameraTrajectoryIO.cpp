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

#include "PinholeCameraTrajectoryIO.h"

#include <unordered_map>
#include <Core/Utility/Console.h>
#include <Core/Utility/FileSystem.h>
#include <IO/ClassIO/IJsonConvertibleIO.h>

namespace open3d {

namespace {

bool ReadPinholeCameraTrajectoryFromJSON(const std::string &filename,
                                         PinholeCameraTrajectory &trajectory) {
    return ReadIJsonConvertible(filename, trajectory);
}

bool WritePinholeCameraTrajectoryToJSON(
        const std::string &filename,
        const PinholeCameraTrajectory &trajectory) {
    return WriteIJsonConvertibleToJSON(filename, trajectory);
}

static const std::unordered_map<
        std::string,
        std::function<bool(const std::string &, PinholeCameraTrajectory &)>>
        file_extension_to_trajectory_read_function{
                {"log", ReadPinholeCameraTrajectoryFromLOG},
                {"json", ReadPinholeCameraTrajectoryFromJSON},
        };

static const std::unordered_map<
        std::string,
        std::function<bool(const std::string &,
                           const PinholeCameraTrajectory &)>>
        file_extension_to_trajectory_write_function{
                {"log", WritePinholeCameraTrajectoryToLOG},
                {"json", WritePinholeCameraTrajectoryToJSON},
        };

}  // unnamed namespace

std::shared_ptr<PinholeCameraTrajectory> CreatePinholeCameraTrajectoryFromFile(
        const std::string &filename) {
    auto trajectory = std::make_shared<PinholeCameraTrajectory>();
    ReadPinholeCameraTrajectory(filename, *trajectory);
    return trajectory;
}

bool ReadPinholeCameraTrajectory(const std::string &filename,
                                 PinholeCameraTrajectory &trajectory) {
    std::string filename_ext =
            filesystem::GetFileExtensionInLowerCase(filename);
    if (filename_ext.empty()) {
        PrintWarning(
                "Read PinholeCameraTrajectory failed: unknown file "
                "extension.\n");
        return false;
    }
    auto map_itr =
            file_extension_to_trajectory_read_function.find(filename_ext);
    if (map_itr == file_extension_to_trajectory_read_function.end()) {
        PrintWarning(
                "Read PinholeCameraTrajectory failed: unknown file "
                "extension.\n");
        return false;
    }
    return map_itr->second(filename, trajectory);
}

bool WritePinholeCameraTrajectory(const std::string &filename,
                                  const PinholeCameraTrajectory &trajectory) {
    std::string filename_ext =
            filesystem::GetFileExtensionInLowerCase(filename);
    if (filename_ext.empty()) {
        PrintWarning(
                "Write PinholeCameraTrajectory failed: unknown file "
                "extension.\n");
        return false;
    }
    auto map_itr =
            file_extension_to_trajectory_write_function.find(filename_ext);
    if (map_itr == file_extension_to_trajectory_write_function.end()) {
        PrintWarning(
                "Write PinholeCameraTrajectory failed: unknown file "
                "extension.\n");
        return false;
    }
    return map_itr->second(filename, trajectory);
}

}  // namespace open3d
