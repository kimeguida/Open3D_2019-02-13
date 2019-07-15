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

#include "SelectionPolygonVolume.h"

#include <json/json.h>
#include <Core/Utility/Console.h>
#include <Core/Geometry/PointCloud.h>
#include <Core/Geometry/TriangleMesh.h>

namespace open3d {

bool SelectionPolygonVolume::ConvertToJsonValue(Json::Value &value) const {
    Json::Value polygon_array;
    for (const auto &point : bounding_polygon_) {
        Json::Value point_object;
        if (EigenVector3dToJsonArray(point, point_object) == false) {
            return false;
        }
        polygon_array.append(point_object);
    }
    value["class_name"] = "SelectionPolygonVolume";
    value["version_major"] = 1;
    value["version_minor"] = 0;
    value["bounding_polygon"] = polygon_array;
    value["orthogonal_axis"] = orthogonal_axis_;
    value["axis_min"] = axis_min_;
    value["axis_max"] = axis_max_;
    return true;
}

bool SelectionPolygonVolume::ConvertFromJsonValue(const Json::Value &value) {
    if (value.isObject() == false) {
        PrintWarning(
                "SelectionPolygonVolume read JSON failed: unsupported json "
                "format.\n");
        return false;
    }
    if (value.get("class_name", "").asString() != "SelectionPolygonVolume" ||
        value.get("version_major", 1).asInt() != 1 ||
        value.get("version_minor", 0).asInt() != 0) {
        PrintWarning(
                "SelectionPolygonVolume read JSON failed: unsupported json "
                "format.\n");
        return false;
    }
    orthogonal_axis_ = value.get("orthogonal_axis", "").asString();
    axis_min_ = value.get("axis_min", 0.0).asDouble();
    axis_max_ = value.get("axis_max", 0.0).asDouble();
    const Json::Value &polygon_array = value["bounding_polygon"];
    if (polygon_array.size() == 0) {
        PrintWarning(
                "SelectionPolygonVolume read JSON failed: empty trajectory.\n");
        return false;
    }
    bounding_polygon_.resize(polygon_array.size());
    for (int i = 0; i < (int)polygon_array.size(); i++) {
        const Json::Value &point_object = polygon_array[i];
        if (EigenVector3dFromJsonArray(bounding_polygon_[i], point_object) ==
            false) {
            return false;
        }
    }
    return true;
}

std::shared_ptr<PointCloud> SelectionPolygonVolume::CropPointCloud(
        const PointCloud &input) const {
    if (orthogonal_axis_ == "" || bounding_polygon_.empty())
        return std::make_shared<PointCloud>();
    return CropPointCloudInPolygon(input);
}

std::shared_ptr<PointCloud> SelectionPolygonVolume::CropPointCloudInPolygon(
        const PointCloud &input) const {
    return SelectDownSample(input, CropInPolygon(input.points_));
}

std::shared_ptr<TriangleMesh> SelectionPolygonVolume::CropTriangleMesh(
        const TriangleMesh &input) const {
    if (orthogonal_axis_ == "" || bounding_polygon_.empty())
        return std::make_shared<TriangleMesh>();
    if (input.HasVertices() && !input.HasTriangles()) {
        PrintWarning(
                "TriangleMesh contains vertices, but no triangles; "
                "cropping will always yield an empty TriangleMesh.\n");
        return std::make_shared<TriangleMesh>();
    }
    return CropTriangleMeshInPolygon(input);
}

std::shared_ptr<TriangleMesh> SelectionPolygonVolume::CropTriangleMeshInPolygon(
        const TriangleMesh &input) const {
    return SelectDownSample(input, CropInPolygon(input.vertices_));
}

std::vector<size_t> SelectionPolygonVolume::CropInPolygon(
        const std::vector<Eigen::Vector3d> &input) const {
    std::vector<size_t> output_index;
    int u, v, w;
    if (orthogonal_axis_ == "x" || orthogonal_axis_ == "X") {
        u = 1;
        v = 2;
        w = 0;
    } else if (orthogonal_axis_ == "y" || orthogonal_axis_ == "Y") {
        u = 0;
        v = 2;
        w = 1;
    } else {
        u = 0;
        v = 1;
        w = 2;
    }
    std::vector<double> nodes;
    ResetConsoleProgress((int64_t)input.size(), "Cropping geometry: ");
    for (size_t k = 0; k < input.size(); k++) {
        AdvanceConsoleProgress();
        const auto &point = input[k];
        if (point(w) < axis_min_ || point(w) > axis_max_) continue;
        nodes.clear();
        for (size_t i = 0; i < bounding_polygon_.size(); i++) {
            size_t j = (i + 1) % bounding_polygon_.size();
            if ((bounding_polygon_[i](v) < point(v) &&
                 bounding_polygon_[j](v) >= point(v)) ||
                (bounding_polygon_[j](v) < point(v) &&
                 bounding_polygon_[i](v) >= point(v))) {
                nodes.push_back(bounding_polygon_[i](u) +
                                (point(v) - bounding_polygon_[i](v)) /
                                        (bounding_polygon_[j](v) -
                                         bounding_polygon_[i](v)) *
                                        (bounding_polygon_[j](u) -
                                         bounding_polygon_[i](u)));
            }
        }
        std::sort(nodes.begin(), nodes.end());
        auto loc = std::lower_bound(nodes.begin(), nodes.end(), point(u));
        if (std::distance(nodes.begin(), loc) % 2 == 1) {
            output_index.push_back(k);
        }
    }
    return output_index;
}

}  // namespace open3d
