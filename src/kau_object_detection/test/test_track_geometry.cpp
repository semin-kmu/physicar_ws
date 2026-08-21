// Copyright 2026 KAU AMET Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "kau_object_detection/track_geometry.hpp"

namespace
{

using kau_object_detection::Point2D;
using kau_object_detection::TrackRing;
using kau_object_detection::TrackRoiOptions;

/// Axis-aligned square centred on the origin, counter-clockwise.
std::vector<Point2D> make_square(const double half_size_m)
{
  return {
    Point2D{-half_size_m, -half_size_m},
    Point2D{half_size_m, -half_size_m},
    Point2D{half_size_m, half_size_m},
    Point2D{-half_size_m, half_size_m},
  };
}

/// Same square with the first point repeated, matching the closed polygons the
/// published route API returns.
std::vector<Point2D> make_closed_square(const double half_size_m)
{
  auto polygon = make_square(half_size_m);
  polygon.push_back(polygon.front());
  return polygon;
}

TrackRoiOptions strict_options()
{
  TrackRoiOptions options;
  options.boundary_tolerance_m = 0.0;
  return options;
}

TEST(TrackGeometry, PointInPolygonSeparatesInsideFromOutside)
{
  const auto square = make_square(2.0);

  EXPECT_TRUE(kau_object_detection::point_in_polygon(square, Point2D{0.0, 0.0}));
  EXPECT_TRUE(kau_object_detection::point_in_polygon(square, Point2D{1.9, 1.9}));
  EXPECT_FALSE(kau_object_detection::point_in_polygon(square, Point2D{2.1, 0.0}));
  EXPECT_FALSE(kau_object_detection::point_in_polygon(square, Point2D{0.0, -5.0}));
  EXPECT_FALSE(kau_object_detection::point_in_polygon(square, Point2D{-3.0, 3.0}));
}

TEST(TrackGeometry, ClosedAndOpenPolygonsBehaveTheSame)
{
  const auto open_square = make_square(2.0);
  const auto closed_square = make_closed_square(2.0);

  for (const auto & point :
    {Point2D{0.0, 0.0}, Point2D{1.5, -1.5}, Point2D{2.5, 0.0}, Point2D{0.0, 9.0}})
  {
    EXPECT_EQ(
      kau_object_detection::point_in_polygon(open_square, point),
      kau_object_detection::point_in_polygon(closed_square, point));
    EXPECT_NEAR(
      kau_object_detection::distance_to_polygon_boundary_m(open_square, point),
      kau_object_detection::distance_to_polygon_boundary_m(closed_square, point),
      1e-12);
  }
}

TEST(TrackGeometry, DegeneratePolygonsAreRejected)
{
  EXPECT_FALSE(kau_object_detection::point_in_polygon({}, Point2D{0.0, 0.0}));
  EXPECT_FALSE(
    kau_object_detection::point_in_polygon(
      {Point2D{0.0, 0.0}, Point2D{1.0, 1.0}}, Point2D{0.5, 0.5}));
  EXPECT_FALSE(
    std::isfinite(
      kau_object_detection::distance_to_polygon_boundary_m({}, Point2D{0.0, 0.0})));

  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    kau_object_detection::point_in_polygon(make_square(1.0), Point2D{nan_value, 0.0}));
}

TEST(TrackGeometry, DistanceToBoundaryMeasuresFromTheNearestEdge)
{
  const auto square = make_square(2.0);

  EXPECT_NEAR(
    kau_object_detection::distance_to_polygon_boundary_m(square, Point2D{0.0, 0.0}), 2.0, 1e-12);
  EXPECT_NEAR(
    kau_object_detection::distance_to_polygon_boundary_m(square, Point2D{1.5, 0.0}), 0.5, 1e-12);
  EXPECT_NEAR(
    kau_object_detection::distance_to_polygon_boundary_m(square, Point2D{2.5, 0.0}), 0.5, 1e-12);
  EXPECT_NEAR(
    kau_object_detection::distance_to_polygon_boundary_m(square, Point2D{2.0, 0.0}), 0.0, 1e-12);
  // Nearest feature is the corner, not an edge interior.
  EXPECT_NEAR(
    kau_object_detection::distance_to_polygon_boundary_m(square, Point2D{5.0, 6.0}),
    std::hypot(3.0, 4.0), 1e-12);
}

TEST(TrackGeometry, RingKeepsTheAnnulusAndRejectsTheHole)
{
  const auto ring =
    kau_object_detection::make_track_ring(make_square(4.0), make_square(1.0));
  ASSERT_TRUE(ring.valid);

  // Inside outer and outside inner.
  EXPECT_TRUE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{2.5, 0.0}, strict_options()));
  EXPECT_TRUE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{0.0, -2.5}, strict_options()));

  // The hole is not drivable.
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{0.0, 0.0}, strict_options()));
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{0.5, 0.5}, strict_options()));

  // Outside the outer boundary.
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{5.0, 0.0}, strict_options()));
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{0.0, 12.0}, strict_options()));
}

TEST(TrackGeometry, BoundaryPointsAreKeptConservatively)
{
  const auto ring =
    kau_object_detection::make_track_ring(make_square(4.0), make_square(1.0));
  ASSERT_TRUE(ring.valid);

  TrackRoiOptions tolerant;
  tolerant.boundary_tolerance_m = 0.15;

  // Just outside the outer boundary but within the keep band.
  EXPECT_TRUE(kau_object_detection::point_is_in_track_ring(ring, Point2D{4.1, 0.0}, tolerant));
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{4.1, 0.0}, strict_options()));

  // Just inside the inner boundary but within the keep band.
  EXPECT_TRUE(kau_object_detection::point_is_in_track_ring(ring, Point2D{0.9, 0.0}, tolerant));
  EXPECT_FALSE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{0.9, 0.0}, strict_options()));

  // Exactly on either boundary.
  EXPECT_TRUE(kau_object_detection::point_is_in_track_ring(ring, Point2D{4.0, 0.0}, tolerant));
  EXPECT_TRUE(kau_object_detection::point_is_in_track_ring(ring, Point2D{1.0, 0.0}, tolerant));

  // Far outside stays rejected even with the tolerance.
  EXPECT_FALSE(kau_object_detection::point_is_in_track_ring(ring, Point2D{6.0, 0.0}, tolerant));
  // Deep inside the hole stays rejected.
  EXPECT_FALSE(kau_object_detection::point_is_in_track_ring(ring, Point2D{0.0, 0.0}, tolerant));
}

TEST(TrackGeometry, InvalidRingRejectsEverything)
{
  const TrackRing empty_ring;
  EXPECT_FALSE(empty_ring.valid);
  EXPECT_FALSE(kau_object_detection::point_is_in_track_ring(empty_ring, Point2D{0.0, 0.0}));

  EXPECT_FALSE(kau_object_detection::make_track_ring({}, make_square(1.0)).valid);
  EXPECT_FALSE(kau_object_detection::make_track_ring(make_square(4.0), {}).valid);
}

TEST(TrackGeometry, RingIsBuiltFromParallelCoordinateArrays)
{
  const std::vector<double> outer_x{-4.0, 4.0, 4.0, -4.0};
  const std::vector<double> outer_y{-4.0, -4.0, 4.0, 4.0};
  const std::vector<double> inner_x{-1.0, 1.0, 1.0, -1.0};
  const std::vector<double> inner_y{-1.0, -1.0, 1.0, 1.0};

  const auto ring =
    kau_object_detection::make_track_ring(outer_x, outer_y, inner_x, inner_y);

  ASSERT_TRUE(ring.valid);
  EXPECT_EQ(ring.outer.size(), 4U);
  EXPECT_EQ(ring.inner.size(), 4U);
  EXPECT_TRUE(
    kau_object_detection::point_is_in_track_ring(ring, Point2D{2.5, 0.0}, strict_options()));

  // Mismatched array lengths cannot form a ring.
  EXPECT_FALSE(
    kau_object_detection::make_track_ring({0.0, 1.0}, outer_y, inner_x, inner_y).valid);
  EXPECT_FALSE(
    kau_object_detection::make_track_ring(outer_x, outer_y, inner_x, {0.0}).valid);
}

TEST(TrackGeometry, NonPositiveToleranceFallsBackToTheStrictTest)
{
  const auto ring =
    kau_object_detection::make_track_ring(make_square(4.0), make_square(1.0));

  TrackRoiOptions negative;
  negative.boundary_tolerance_m = -1.0;
  EXPECT_FALSE(kau_object_detection::point_is_in_track_ring(ring, Point2D{4.1, 0.0}, negative));
  EXPECT_TRUE(kau_object_detection::point_is_in_track_ring(ring, Point2D{2.0, 0.0}, negative));

  TrackRoiOptions non_finite;
  non_finite.boundary_tolerance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kau_object_detection::point_is_in_track_ring(ring, Point2D{4.1, 0.0}, non_finite));
}

}  // namespace
