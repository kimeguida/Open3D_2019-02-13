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

#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

bool ReadLogFile(const std::string &filename,
                 std::vector<std::tuple<int, int, int>> &metadata,
                 std::vector<Eigen::Matrix4d> &transformations) {
    using namespace open3d;
    metadata.clear();
    transformations.clear();
    FILE *f = fopen(filename.c_str(), "r");
    if (f == NULL) {
        PrintWarning("Read LOG failed: unable to open file.\n");
        return false;
    }
    char line_buffer[DEFAULT_IO_BUFFER_SIZE];
    int i, j, k;
    Eigen::Matrix4d trans;
    while (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, f)) {
        if (strlen(line_buffer) > 0 && line_buffer[0] != '#') {
            if (sscanf(line_buffer, "%d %d %d", &i, &j, &k) != 3) {
                PrintWarning("Read LOG failed: unrecognized format.\n");
                return false;
            }
            if (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, f) == 0) {
                PrintWarning("Read LOG failed: unrecognized format.\n");
                return false;
            } else {
                sscanf(line_buffer, "%lf %lf %lf %lf", &trans(0, 0),
                       &trans(0, 1), &trans(0, 2), &trans(0, 3));
            }
            if (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, f) == 0) {
                PrintWarning("Read LOG failed: unrecognized format.\n");
                return false;
            } else {
                sscanf(line_buffer, "%lf %lf %lf %lf", &trans(1, 0),
                       &trans(1, 1), &trans(1, 2), &trans(1, 3));
            }
            if (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, f) == 0) {
                PrintWarning("Read LOG failed: unrecognized format.\n");
                return false;
            } else {
                sscanf(line_buffer, "%lf %lf %lf %lf", &trans(2, 0),
                       &trans(2, 1), &trans(2, 2), &trans(2, 3));
            }
            if (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, f) == 0) {
                PrintWarning("Read LOG failed: unrecognized format.\n");
                return false;
            } else {
                sscanf(line_buffer, "%lf %lf %lf %lf", &trans(3, 0),
                       &trans(3, 1), &trans(3, 2), &trans(3, 3));
            }
            metadata.push_back(std::make_tuple(i, j, k));
            transformations.push_back(trans);
        }
    }
    fclose(f);
    return true;
}

void PrintHelp() {
    using namespace open3d;
    PrintOpen3DVersion();
    // clang-format off
    PrintInfo("Usage:\n");
    PrintInfo("    > ViewPCDMatch [options]\n");
    PrintInfo("      View pairwise matching result of point clouds.\n");
    PrintInfo("\n");
    PrintInfo("Basic options:\n");
    PrintInfo("    --help, -h                : Print help information.\n");
    PrintInfo("    --log file                : A log file of the pairwise matching results. Must have.\n");
    PrintInfo("    --dir directory           : The directory storing all pcd files. By default it is the parent directory of the log file + pcd/.\n");
    PrintInfo("    --verbose n               : Set verbose level (0-4). Default: 2.\n");
    // clang-format on
}

int main(int argc, char *argv[]) {
    using namespace open3d;

    if (argc <= 1 || ProgramOptionExists(argc, argv, "--help") ||
        ProgramOptionExists(argc, argv, "-h")) {
        PrintHelp();
        return 1;
    }
    const int NUM_OF_COLOR_PALETTE = 5;
    Eigen::Vector3d color_palette[NUM_OF_COLOR_PALETTE] = {
            Eigen::Vector3d(255, 180, 0) / 255.0,
            Eigen::Vector3d(0, 166, 237) / 255.0,
            Eigen::Vector3d(246, 81, 29) / 255.0,
            Eigen::Vector3d(127, 184, 0) / 255.0,
            Eigen::Vector3d(13, 44, 84) / 255.0,
    };

    int verbose = GetProgramOptionAsInt(argc, argv, "--verbose", 2);
    SetVerbosityLevel((VerbosityLevel)verbose);
    std::string log_filename = GetProgramOptionAsString(argc, argv, "--log");
    std::string pcd_dirname = GetProgramOptionAsString(argc, argv, "--dir");
    if (pcd_dirname.empty()) {
        pcd_dirname =
                filesystem::GetFileParentDirectory(log_filename) + "pcds/";
    }

    std::vector<std::tuple<int, int, int>> metadata;
    std::vector<Eigen::Matrix4d> transformations;
    ReadLogFile(log_filename, metadata, transformations);

    for (auto k = 0; k < metadata.size(); k++) {
        auto i = std::get<0>(metadata[k]), j = std::get<1>(metadata[k]);
        PrintInfo("Showing matched point cloud #%d and #%d.\n", i, j);
        auto pcd_target = CreatePointCloudFromFile(pcd_dirname + "cloud_bin_" +
                                                   std::to_string(i) + ".pcd");
        pcd_target->colors_.clear();
        pcd_target->colors_.resize(pcd_target->points_.size(),
                                   color_palette[0]);
        auto pcd_source = CreatePointCloudFromFile(pcd_dirname + "cloud_bin_" +
                                                   std::to_string(j) + ".pcd");
        pcd_source->colors_.clear();
        pcd_source->colors_.resize(pcd_source->points_.size(),
                                   color_palette[1]);
        pcd_source->Transform(transformations[k]);
        DrawGeometriesWithCustomAnimation({pcd_target, pcd_source},
                                          "ViewPCDMatch", 1600, 900);
    }
    return 0;
}
