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

#include "Utility/UnitTest.h"
#include "Utility/Raw.h"

#include "Core/Geometry/LineSet.h"
#include "Core/Geometry/PointCloud.h"

using namespace Eigen;
using namespace open3d;
using namespace std;
using namespace unit_test;

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, Constructor) {
    LineSet ls;

    // inherited from Geometry2D
    EXPECT_EQ(Geometry::GeometryType::LineSet, ls.GetGeometryType());
    EXPECT_EQ(3, ls.Dimension());

    // public member variables
    EXPECT_EQ(0, ls.points_.size());
    EXPECT_EQ(0, ls.lines_.size());
    EXPECT_EQ(0, ls.colors_.size());

    // public members
    EXPECT_TRUE(ls.IsEmpty());

    ExpectEQ(Zero3d, ls.GetMinBound());
    ExpectEQ(Zero3d, ls.GetMaxBound());

    EXPECT_FALSE(ls.HasPoints());
    EXPECT_FALSE(ls.HasLines());
    EXPECT_FALSE(ls.HasColors());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, DISABLED_MemberData) { unit_test::NotImplemented(); }

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, Clear) {
    int size = 100;

    Vector3d dmin(0.0, 0.0, 0.0);
    Vector3d dmax(1000.0, 1000.0, 1000.0);

    Vector2i imin(0, 0);
    Vector2i imax(1000, 1000);

    LineSet ls;

    ls.points_.resize(size);
    ls.lines_.resize(size);
    ls.colors_.resize(size);

    Rand(ls.points_, dmin, dmax, 0);
    Rand(ls.lines_, imin, imax, 0);
    Rand(ls.colors_, dmin, dmax, 0);

    EXPECT_FALSE(ls.IsEmpty());

    ExpectEQ(Vector3d(19.607843, 0.0, 0.0), ls.GetMinBound());
    ExpectEQ(Vector3d(996.078431, 996.078431, 996.078431), ls.GetMaxBound());

    EXPECT_TRUE(ls.HasPoints());
    EXPECT_TRUE(ls.HasLines());
    EXPECT_TRUE(ls.HasColors());

    ls.Clear();

    // public members
    EXPECT_TRUE(ls.IsEmpty());
    ExpectEQ(Zero3d, ls.GetMinBound());
    ExpectEQ(Zero3d, ls.GetMaxBound());

    EXPECT_FALSE(ls.HasPoints());
    EXPECT_FALSE(ls.HasLines());
    EXPECT_FALSE(ls.HasColors());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, IsEmpty) {
    int size = 100;

    Vector3d vmin(0.0, 0.0, 0.0);
    Vector3d vmax(1000.0, 1000.0, 1000.0);

    LineSet ls;

    EXPECT_TRUE(ls.IsEmpty());

    ls.points_.resize(size);

    Rand(ls.points_, vmin, vmax, 0);

    EXPECT_FALSE(ls.IsEmpty());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, GetMinBound) {
    int size = 100;

    Vector3d vmin(0.0, 0.0, 0.0);
    Vector3d vmax(1000.0, 1000.0, 1000.0);

    LineSet ls;

    ls.points_.resize(size);

    Rand(ls.points_, vmin, vmax, 0);

    Vector3d minBound = ls.GetMinBound();

    ExpectEQ(Vector3d(19.607843, 0.0, 0.0), ls.GetMinBound());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, GetMaxBound) {
    int size = 100;

    Vector3d vmin(0.0, 0.0, 0.0);
    Vector3d vmax(1000.0, 1000.0, 1000.0);

    LineSet ls;

    ls.points_.resize(size);

    Rand(ls.points_, vmin, vmax, 0);

    Vector3d maxBound = ls.GetMaxBound();

    ExpectEQ(Vector3d(996.078431, 996.078431, 996.078431), ls.GetMaxBound());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, Transform) {
    vector<Vector3d> ref_points = {{396.870588, 1201.976471, 880.472941},
                                   {320.792157, 1081.976471, 829.139608},
                                   {269.027451, 818.447059, 406.786667},
                                   {338.831373, 1001.192157, 614.237647},
                                   {423.537255, 1153.349020, 483.727843},
                                   {432.949020, 1338.447059, 964.512157},
                                   {140.007843, 444.721569, 189.296471},
                                   {292.164706, 763.152941, 317.178824},
                                   {134.517647, 407.858824, 192.002353},
                                   {274.909804, 802.368627, 218.747451}};

    vector<Vector2i> ref_lines = {
            {839, 392}, {780, 796}, {909, 196}, {333, 764}, {274, 552},
            {474, 627}, {364, 509}, {949, 913}, {635, 713}, {141, 603}};

    int size = 10;
    LineSet ls;

    Vector3d dmin(0.0, 0.0, 0.0);
    Vector3d dmax(1000.0, 1000.0, 1000.0);

    Vector2i imin(0, 0);
    Vector2i imax(1000, 1000);

    ls.points_.resize(size);
    Rand(ls.points_, dmin, dmax, 0);

    ls.lines_.resize(size);
    Rand(ls.lines_, imin, imax, 0);

    Matrix4d transformation;
    transformation << 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90,
            0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16;

    ls.Transform(transformation);

    ExpectEQ(ref_points, ls.points_);
    ExpectEQ(ref_lines, ls.lines_);
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, OperatorAppend) {
    int size = 100;

    LineSet ls0;
    LineSet ls1;

    ls0.points_.resize(size);
    ls0.lines_.resize(size);
    ls0.colors_.resize(size);

    ls1.points_.resize(size);
    ls1.lines_.resize(size);
    ls1.colors_.resize(size);

    Rand(ls0.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(ls0.lines_, Zero2i, Vector2i(size - 1, size - 1), 0);
    Rand(ls0.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 0);

    Rand(ls1.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(ls1.lines_, Zero2i, Vector2i(size - 1, size - 1), 0);
    Rand(ls1.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 1);

    vector<Vector3d> p;
    p.insert(p.end(), ls0.points_.begin(), ls0.points_.end());
    p.insert(p.end(), ls1.points_.begin(), ls1.points_.end());

    vector<Vector2i> n;
    n.insert(n.end(), ls0.lines_.begin(), ls0.lines_.end());
    n.insert(n.end(), ls1.lines_.begin(), ls1.lines_.end());

    vector<Vector3d> c;
    c.insert(c.end(), ls0.colors_.begin(), ls0.colors_.end());
    c.insert(c.end(), ls1.colors_.begin(), ls1.colors_.end());

    LineSet ls(ls0);
    ls += ls1;

    EXPECT_EQ(2 * size, ls.points_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.points_[i], ls.points_[i + 0]);
        ExpectEQ(ls1.points_[i], ls.points_[i + size]);
    }

    EXPECT_EQ(2 * size, ls.lines_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.lines_[i], ls.lines_[i + 0]);

        Vector2i ls1_line_i = {ls1.lines_[i](0, 0) + size,
                               ls1.lines_[i](1, 0) + size};
        ExpectEQ(ls1_line_i, ls.lines_[i + ls0.lines_.size()]);
    }

    EXPECT_EQ(2 * size, ls.colors_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.colors_[i], ls.colors_[i + 0]);
        ExpectEQ(ls1.colors_[i], ls.colors_[i + size]);
    }
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, OperatorADD) {
    int size = 100;

    LineSet ls0;
    LineSet ls1;

    ls0.points_.resize(size);
    ls0.lines_.resize(size);
    ls0.colors_.resize(size);

    ls1.points_.resize(size);
    ls1.lines_.resize(size);
    ls1.colors_.resize(size);

    Rand(ls0.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(ls0.lines_, Zero2i, Vector2i(size - 1, size - 1), 0);
    Rand(ls0.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 0);

    Rand(ls1.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(ls1.lines_, Zero2i, Vector2i(size - 1, size - 1), 0);
    Rand(ls1.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 1);

    vector<Vector3d> p;
    p.insert(p.end(), ls0.points_.begin(), ls0.points_.end());
    p.insert(p.end(), ls1.points_.begin(), ls1.points_.end());

    vector<Vector2i> n;
    n.insert(n.end(), ls0.lines_.begin(), ls0.lines_.end());
    n.insert(n.end(), ls1.lines_.begin(), ls1.lines_.end());

    vector<Vector3d> c;
    c.insert(c.end(), ls0.colors_.begin(), ls0.colors_.end());
    c.insert(c.end(), ls1.colors_.begin(), ls1.colors_.end());

    LineSet ls = ls0 + ls1;

    EXPECT_EQ(2 * size, ls.points_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.points_[i], ls.points_[i + 0]);
        ExpectEQ(ls1.points_[i], ls.points_[i + size]);
    }

    EXPECT_EQ(2 * size, ls.lines_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.lines_[i], ls.lines_[i + 0]);

        Vector2i ls1_line_i = {ls1.lines_[i](0, 0) + size,
                               ls1.lines_[i](1, 0) + size};
        ExpectEQ(ls1_line_i, ls.lines_[i + ls0.lines_.size()]);
    }

    EXPECT_EQ(2 * size, ls.colors_.size());
    for (size_t i = 0; i < size; i++) {
        ExpectEQ(ls0.colors_[i], ls.colors_[i + 0]);
        ExpectEQ(ls1.colors_[i], ls.colors_[i + size]);
    }
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, HasPoints) {
    int size = 100;

    LineSet ls;

    EXPECT_FALSE(ls.HasPoints());

    ls.points_.resize(size);

    EXPECT_TRUE(ls.HasPoints());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, HasLines) {
    int size = 100;

    LineSet ls;

    EXPECT_FALSE(ls.HasLines());

    ls.points_.resize(size);
    ls.lines_.resize(size);

    EXPECT_TRUE(ls.HasLines());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, HasColors) {
    int size = 100;

    LineSet ls;

    EXPECT_FALSE(ls.HasColors());

    ls.points_.resize(size);
    ls.lines_.resize(size);
    ls.colors_.resize(size);

    EXPECT_TRUE(ls.HasColors());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, GetLineCoordinate) {
    vector<vector<Vector3d>> ref_points = {
            {{239.215686, 133.333333, 803.921569},
             {552.941176, 474.509804, 627.450980}},
            {{239.215686, 133.333333, 803.921569},
             {239.215686, 133.333333, 803.921569}},
            {{152.941176, 400.000000, 129.411765},
             {796.078431, 909.803922, 196.078431}},
            {{552.941176, 474.509804, 627.450980},
             {141.176471, 603.921569, 15.686275}},
            {{333.333333, 764.705882, 274.509804},
             {364.705882, 509.803922, 949.019608}},
            {{364.705882, 509.803922, 949.019608},
             {913.725490, 635.294118, 713.725490}},
            {{552.941176, 474.509804, 627.450980},
             {364.705882, 509.803922, 949.019608}},
            {{152.941176, 400.000000, 129.411765},
             {152.941176, 400.000000, 129.411765}},
            {{913.725490, 635.294118, 713.725490},
             {141.176471, 603.921569, 15.686275}},
            {{796.078431, 909.803922, 196.078431},
             {913.725490, 635.294118, 713.725490}}};

    int size = 10;
    LineSet ls;

    Vector3d dmin(0.0, 0.0, 0.0);
    Vector3d dmax(1000.0, 1000.0, 1000.0);

    Vector2i imin(0, 0);
    Vector2i imax(size - 1, size - 1);

    ls.points_.resize(size);
    Rand(ls.points_, dmin, dmax, 0);

    ls.lines_.resize(size);
    Rand(ls.lines_, imin, imax, 0);

    EXPECT_EQ(ref_points.size(), size);
    for (size_t i = 0; i < size; i++) {
        auto result = ls.GetLineCoordinate(i);

        ExpectEQ(ref_points[i][0], result.first);
        ExpectEQ(ref_points[i][1], result.second);
    }
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(LineSet, CreateLineSetFromPointCloudCorrespondences) {
    int size = 10;

    vector<Vector3d> ref_points = {{839.215686, 392.156863, 780.392157},
                                   {796.078431, 909.803922, 196.078431},
                                   {333.333333, 764.705882, 274.509804},
                                   {552.941176, 474.509804, 627.450980},
                                   {364.705882, 509.803922, 949.019608},
                                   {913.725490, 635.294118, 713.725490},
                                   {141.176471, 603.921569, 15.686275},
                                   {239.215686, 133.333333, 803.921569},
                                   {152.941176, 400.000000, 129.411765},
                                   {105.882353, 996.078431, 215.686275},
                                   {839.215686, 392.156863, 780.392157},
                                   {796.078431, 909.803922, 196.078431},
                                   {333.333333, 764.705882, 274.509804},
                                   {552.941176, 474.509804, 627.450980},
                                   {364.705882, 509.803922, 949.019608},
                                   {913.725490, 635.294118, 713.725490},
                                   {141.176471, 603.921569, 15.686275},
                                   {239.215686, 133.333333, 803.921569},
                                   {152.941176, 400.000000, 129.411765},
                                   {105.882353, 996.078431, 215.686275}};

    vector<Vector2i> ref_lines = {{8, 13}, {7, 17}, {9, 11}, {3, 17}, {2, 15},
                                  {4, 16}, {3, 15}, {9, 19}, {6, 17}, {1, 16}};

    PointCloud pc0;
    PointCloud pc1;
    vector<pair<int, int>> correspondence(size);

    pc0.points_.resize(size);
    pc0.normals_.resize(size);
    pc0.colors_.resize(size);

    pc1.points_.resize(size);
    pc1.normals_.resize(size);
    pc1.colors_.resize(size);

    Rand(pc0.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc0.normals_, Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc0.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 0);

    Rand(pc1.points_, Zero3d, Vector3d(1000.0, 1000.0, 1000.0), 0);
    Rand(pc1.normals_, Vector3d(-1.0, -1.0, -1.0), Vector3d(1.0, 1.0, 1.0), 0);
    Rand(pc1.colors_, Zero3d, Vector3d(1.0, 1.0, 1.0), 1);

    Raw raw;
    for (int i = 0; i < size; i++) {
        int first = size * raw.Next<int>() / Raw::VMAX;
        int second = size * raw.Next<int>() / Raw::VMAX;

        correspondence[i] = pair<int, int>(first, second);
    }

    auto ls = CreateLineSetFromPointCloudCorrespondences(pc0, pc1,
                                                         correspondence);

    ExpectEQ(ref_points, ls->points_);
    ExpectEQ(ref_lines, ls->lines_);
}
