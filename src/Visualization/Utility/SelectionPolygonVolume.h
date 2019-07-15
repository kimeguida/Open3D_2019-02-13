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

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <Core/Utility/IJsonConvertible.h>

namespace open3d {

class Geometry;
class PointCloud;
class TriangleMesh;

class SelectionPolygonVolume : public IJsonConvertible {
public:
    ~SelectionPolygonVolume() override {}

public:
    bool ConvertToJsonValue(Json::Value &value) const override;
    bool ConvertFromJsonValue(const Json::Value &value) override;
    std::shared_ptr<PointCloud> CropPointCloud(const PointCloud &input) const;
    std::shared_ptr<TriangleMesh> CropTriangleMesh(
            const TriangleMesh &input) const;

private:
    std::shared_ptr<PointCloud> CropPointCloudInPolygon(
            const PointCloud &input) const;
    std::shared_ptr<TriangleMesh> CropTriangleMeshInPolygon(
            const TriangleMesh &input) const;
    std::vector<size_t> CropInPolygon(
            const std::vector<Eigen::Vector3d> &input) const;

public:
    std::string orthogonal_axis_ = "";
    std::vector<Eigen::Vector3d> bounding_polygon_;
    double axis_min_ = 0.0;
    double axis_max_ = 0.0;
};

}  // namespace open3d
