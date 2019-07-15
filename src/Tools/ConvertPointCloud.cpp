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

#include <Core/Core.h>
#include <IO/IO.h>

#include <limits>

void PrintHelp() {
    using namespace open3d;
    PrintOpen3DVersion();
    // clang-format off
    PrintInfo("Usage:\n");
    PrintInfo("    > ConvertPointCloud source_file target_file [options]\n");
    PrintInfo("    > ConvertPointCloud source_directory target_directory [options]\n");
    PrintInfo("      Read point cloud from source file and convert it to target file.\n");
    PrintInfo("\n");
    PrintInfo("Options (listed in the order of execution priority):\n");
    PrintInfo("    --help, -h                : Print help information.\n");
    PrintInfo("    --verbose n               : Set verbose level (0-4).\n");
    PrintInfo("    --clip_x_min x0           : Clip points with x coordinate < x0.\n");
    PrintInfo("    --clip_x_max x1           : Clip points with x coordinate > x1.\n");
    PrintInfo("    --clip_y_min y0           : Clip points with y coordinate < y0.\n");
    PrintInfo("    --clip_y_max y1           : Clip points with y coordinate > y1.\n");
    PrintInfo("    --clip_z_min z0           : Clip points with z coordinate < z0.\n");
    PrintInfo("    --clip_z_max z1           : Clip points with z coordinate > z1.\n");
    PrintInfo("    --filter_mahalanobis d    : Filter out points with Mahalanobis distance > d.\n");
    PrintInfo("    --uniform_sample_every K  : Downsample the point cloud uniformly. Keep only\n");
    PrintInfo("                              : one point for every K points.\n");
    PrintInfo("    --voxel_sample voxel_size : Downsample the point cloud with a voxel.\n");
    PrintInfo("    --estimate_normals radius : Estimate normals using a search neighborhood of\n");
    PrintInfo("                                radius. The normals are oriented w.r.t. the\n");
    PrintInfo("                                original normals of the pointcloud if they\n");
    PrintInfo("                                exist. Otherwise, they are oriented towards -Z\n");
    PrintInfo("                                direction.\n");
    PrintInfo("    --estimate_normals_knn k  : Estimate normals using a search with k nearest\n");
    PrintInfo("                                neighbors. The normals are oriented w.r.t. the\n");
    PrintInfo("                                original normals of the pointcloud if they\n");
    PrintInfo("                                exist. Otherwise, they are oriented towards -Z\n");
    PrintInfo("                                direction.\n");
    PrintInfo("    --orient_normals [x,y,z]  : Orient the normals w.r.t the direction [x,y,z].\n");
    PrintInfo("    --camera_location [x,y,z] : Orient the normals w.r.t camera location [x,y,z].\n");
    // clang-format on
}

void convert(int argc,
             char **argv,
             const std::string &file_in,
             const std::string &file_out) {
    using namespace open3d;
    using namespace open3d::filesystem;
    auto pointcloud_ptr = CreatePointCloudFromFile(file_in.c_str());
    size_t point_num_in = pointcloud_ptr->points_.size();
    bool processed = false;

    // clip
    if (ProgramOptionExistsAny(
                argc, argv,
                {"--clip_x_min", "--clip_x_max", "--clip_y_min", "--clip_y_max",
                 "--clip_z_min", "--clip_z_max"})) {
        Eigen::Vector3d min_bound, max_bound;
        min_bound(0) =
                GetProgramOptionAsDouble(argc, argv, "--clip_x_min",
                                         std::numeric_limits<double>::lowest());
        min_bound(1) =
                GetProgramOptionAsDouble(argc, argv, "--clip_y_min",
                                         std::numeric_limits<double>::lowest());
        min_bound(2) =
                GetProgramOptionAsDouble(argc, argv, "--clip_z_min",
                                         std::numeric_limits<double>::lowest());
        max_bound(0) = GetProgramOptionAsDouble(
                argc, argv, "--clip_x_max", std::numeric_limits<double>::max());
        max_bound(1) = GetProgramOptionAsDouble(
                argc, argv, "--clip_y_max", std::numeric_limits<double>::max());
        max_bound(2) = GetProgramOptionAsDouble(
                argc, argv, "--clip_z_max", std::numeric_limits<double>::max());
        pointcloud_ptr = CropPointCloud(*pointcloud_ptr, min_bound, max_bound);
        processed = true;
    }

    // filter_mahalanobis
    double mahalanobis_threshold =
            GetProgramOptionAsDouble(argc, argv, "--filter_mahalanobis", 0.0);
    if (mahalanobis_threshold > 0.0) {
        auto mahalanobis =
                ComputePointCloudMahalanobisDistance(*pointcloud_ptr);
        std::vector<size_t> indices;
        for (size_t i = 0; i < pointcloud_ptr->points_.size(); i++) {
            if (mahalanobis[i] < mahalanobis_threshold) {
                indices.push_back(i);
            }
        }
        auto pcd = SelectDownSample(*pointcloud_ptr, indices);
        PrintDebug("Based on Mahalanobis distance, %d points were filtered.\n",
                   (int)(pointcloud_ptr->points_.size() - pcd->points_.size()));
        pointcloud_ptr = pcd;
    }

    // uniform_downsample
    int every_k =
            GetProgramOptionAsInt(argc, argv, "--uniform_sample_every", 0);
    if (every_k > 1) {
        PrintDebug("Downsample point cloud uniformly every %d points.\n",
                   every_k);
        pointcloud_ptr = UniformDownSample(*pointcloud_ptr, every_k);
        processed = true;
    }

    // voxel_downsample
    double voxel_size =
            GetProgramOptionAsDouble(argc, argv, "--voxel_sample", 0.0);
    if (voxel_size > 0.0) {
        PrintDebug("Downsample point cloud with voxel size %.4f.\n",
                   voxel_size);
        pointcloud_ptr = VoxelDownSample(*pointcloud_ptr, voxel_size);
        processed = true;
    }

    // estimate_normals
    double radius =
            GetProgramOptionAsDouble(argc, argv, "--estimate_normals", 0.0);
    if (radius > 0.0) {
        PrintDebug("Estimate normals with search radius %.4f.\n", radius);
        EstimateNormals(*pointcloud_ptr, KDTreeSearchParamRadius(radius));
        processed = true;
    }

    int k = GetProgramOptionAsInt(argc, argv, "--estimate_normals_knn", 0);
    if (k > 0) {
        PrintDebug("Estimate normals with search knn %d.\n", k);
        EstimateNormals(*pointcloud_ptr, KDTreeSearchParamKNN(k));
        processed = true;
    }

    // orient_normals
    Eigen::VectorXd direction =
            GetProgramOptionAsEigenVectorXd(argc, argv, "--orient_normals");
    if (direction.size() == 3 && pointcloud_ptr->HasNormals()) {
        PrintDebug("Orient normals to [%.2f, %.2f, %.2f].\n", direction(0),
                   direction(1), direction(2));
        Eigen::Vector3d dir(direction);
        OrientNormalsToAlignWithDirection(*pointcloud_ptr, dir);
        processed = true;
    }
    Eigen::VectorXd camera_loc =
            GetProgramOptionAsEigenVectorXd(argc, argv, "--camera_location");
    if (camera_loc.size() == 3 && pointcloud_ptr->HasNormals()) {
        PrintDebug("Orient normals towards [%.2f, %.2f, %.2f].\n",
                   camera_loc(0), camera_loc(1), camera_loc(2));
        Eigen::Vector3d loc(camera_loc);
        OrientNormalsTowardsCameraLocation(*pointcloud_ptr, loc);
        processed = true;
    }

    size_t point_num_out = pointcloud_ptr->points_.size();
    if (processed) {
        PrintInfo("Processed point cloud from %d points to %d points.\n",
                  (int)point_num_in, (int)point_num_out);
    }
    WritePointCloud(file_out.c_str(), *pointcloud_ptr, false, true);
}

int main(int argc, char **argv) {
    using namespace open3d;
    using namespace open3d::filesystem;

    if (argc < 3 || ProgramOptionExists(argc, argv, "--help") ||
        ProgramOptionExists(argc, argv, "-h")) {
        PrintHelp();
        return 0;
    }

    int verbose = GetProgramOptionAsInt(argc, argv, "--verbose", 2);
    SetVerbosityLevel((VerbosityLevel)verbose);

    if (FileExists(argv[1])) {
        convert(argc, argv, argv[1], argv[2]);
    } else if (DirectoryExists(argv[1])) {
        MakeDirectoryHierarchy(argv[2]);
        std::vector<std::string> filenames;
        ListFilesInDirectory(argv[1], filenames);
        for (const auto &fn : filenames) {
            convert(argc, argv, fn,
                    GetRegularizedDirectoryName(argv[2]) +
                            GetFileNameWithoutDirectory(fn));
        }
    } else {
        PrintError("File or directory does not exist.\n");
    }

    return 1;
}
