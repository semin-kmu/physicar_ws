// 2D LiDAR 클러스터링 · 형상 · 원 근사.
// kau_object_detection 의 laser_scan_clusterer / cluster_geometry /
// cone_occupancy 이식. 헤더 전용 -- CMakeLists 가 이 패키지에 별도 .cpp 를
// 두지 않는다.
//
// 좌표는 전부 lidar_link 기준이다. 인접 임계와 가시면 보정이 센서 원점에
// 의존하므로 base_link 변환은 원 중심이 나온 뒤 노드가 한다.
//
// 처리 순서: 빔 추출 -> 스페클 제거 -> 클러스터링 -> 형상 -> 원 근사

#ifndef BACKUP_OBJECT_DETECTION__CLUSTERING_HPP_
#define BACKUP_OBJECT_DETECTION__CLUSTERING_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace backup_object_detection
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;

struct ScanPoint2D
{
  std::size_t scan_index{0U};
  double x_m{0.0};
  double y_m{0.0};
  double range_m{0.0};
  double angle_rad{0.0};
};

struct Cluster2D
{
  std::vector<ScanPoint2D> points;
  bool wraps_scan_boundary{false};
};

struct ClusteringOptions
{
  double base_distance_threshold_m{0.05};
  double angular_resolution_scale{1.5};
  double circular_scan_tolerance_rad{0.05};
  std::size_t minimum_cluster_points{1U};
  double collinear_tolerance_m{0.001};

  // --- 스페클 필터 ---
  bool speckle_filter_enabled{true};
  /// 1 이면 서로 지지하는 2 빔 후보가 살아남고, 2 면 다시 버려진다.
  std::size_t minimum_support_neighbors{1U};
  /// 클러스터 연결 임계 대비 지지 반경. 1.0 이면 연결을 가진 점은 절대
  /// 스페클로 제거되지 않는다.
  double neighbor_distance_scale{1.0};
};

/// 콘 원 근사 설정.
struct ConeCircleOptions
{
  /// 콘 밑면 0.18 x 0.18 m 의 절반.
  double nominal_cone_radius_m{0.09};
  /// 라이다 장착 높이에서 실제로 보이는 단면의 반경. 관측 표면이 콘 축보다
  /// 이만큼 앞에 있으므로, 최근접점을 센서 반대쪽으로 이만큼 밀어 축을 복원한다.
  double visible_slice_radius_m{0.043};
  /// 이보다 넓게 관측된 클러스터는 벽/트랙 경계다. 콘 크기 원으로 보고하면
  /// 장애물을 과소평가하므로 축소하지 않고 버린다.
  double maximum_observed_width_m{0.30};
};

/// 빔 인접 구조. 720 점 360 deg 스캔이면 마지막 빔 다음이 0 번 빔이다.
struct ScanTopology
{
  std::size_t beam_count{0U};
  double angle_increment_rad{0.0};
  bool wraps_full_circle{false};
  bool valid{false};
};

/// 관측된 클러스터의 형상. 전부 측정값이며 안전 여유가 들어가지 않는다.
struct ClusterGeometry2D
{
  bool valid{false};
  /// range 가 가장 작은 측정점. 원 중심 복원의 기준이다.
  ScanPoint2D closest_point;
  /// 스캔 순서상 첫 점과 끝 점 사이 현 길이.
  double width_m{0.0};
  /// 관측점 두 개 사이의 최대 거리. 극점 쌍은 항상 볼록 껍질 위에 있다.
  double max_extent_m{0.0};
  std::size_t point_count{0U};
};

/// 클러스터 하나에서 유도한 원.
struct ConeCircle
{
  bool valid{false};
  double center_x_m{0.0};
  double center_y_m{0.0};
  /// nominal_cone_radius_m 그대로. 적합(fit)이 아니다.
  double radius_m{0.0};
  double observed_extent_m{0.0};
  std::size_t point_count{0U};
};

inline bool isFiniteFloat(const float value)
{
  return std::isfinite(static_cast<double>(value));
}

inline bool pointIsFinite(const ScanPoint2D & point)
{
  return std::isfinite(point.x_m) && std::isfinite(point.y_m) && std::isfinite(point.range_m);
}

inline double planarDistance(const ScanPoint2D & first, const ScanPoint2D & second)
{
  return std::hypot(second.x_m - first.x_m, second.y_m - first.y_m);
}

inline double wrapPi(const double angle_rad)
{
  double wrapped = std::fmod(angle_rad + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

inline bool hasValidScanGeometry(const sensor_msgs::msg::LaserScan & scan)
{
  return isFiniteFloat(scan.angle_min) &&
         isFiniteFloat(scan.angle_max) &&
         isFiniteFloat(scan.angle_increment) &&
         scan.angle_max > scan.angle_min &&
         scan.angle_increment > 0.0F &&
         isFiniteFloat(scan.range_min) &&
         isFiniteFloat(scan.range_max) &&
         scan.range_min >= 0.0F &&
         scan.range_max > scan.range_min;
}

inline bool scanCoversFullCircle(
  const sensor_msgs::msg::LaserScan & scan,
  const double tolerance_rad)
{
  if (!hasValidScanGeometry(scan) || !std::isfinite(tolerance_rad) || tolerance_rad < 0.0) {
    return false;
  }

  const double covered_angle =
    static_cast<double>(scan.angle_max - scan.angle_min) +
    std::abs(static_cast<double>(scan.angle_increment));
  return std::abs(covered_angle - kTwoPi) <= tolerance_rad;
}

inline ScanTopology makeScanTopology(
  const sensor_msgs::msg::LaserScan & scan,
  const double circular_scan_tolerance_rad)
{
  ScanTopology topology;
  if (!hasValidScanGeometry(scan) || scan.ranges.empty()) {
    return topology;
  }

  topology.beam_count = scan.ranges.size();
  topology.angle_increment_rad = static_cast<double>(scan.angle_increment);
  topology.wraps_full_circle =
    topology.beam_count > 1U &&
    scanCoversFullCircle(scan, circular_scan_tolerance_rad);
  topology.valid = true;
  return topology;
}

/// second_index 가 first_index 의 바로 다음 빔인가. 전주기 스캔에서만
/// 마지막 빔 -> 0 번 빔 을 인정한다.
inline bool scanIndexFollows(
  const std::size_t first_index,
  const std::size_t second_index,
  const ScanTopology & topology)
{
  if (!topology.valid ||
    first_index >= topology.beam_count ||
    second_index >= topology.beam_count)
  {
    return false;
  }

  if (second_index == first_index + 1U) {
    return true;
  }

  return topology.wraps_full_circle &&
         first_index + 1U == topology.beam_count &&
         second_index == 0U;
}

/// 두 빔이 같은 물체로 이어지는 최대 거리 [m].
/// 먼 물체일수록 빔 간격이 벌어지므로 range 에 비례해 임계를 키운다.
/// 입력이 쓸 수 없으면 음수를 돌려준다.
inline double adaptiveNeighborDistance(
  const double first_range_m,
  const double second_range_m,
  const double angle_increment_rad,
  const ClusteringOptions & options)
{
  if (!std::isfinite(first_range_m) ||
    !std::isfinite(second_range_m) ||
    !std::isfinite(angle_increment_rad) ||
    options.base_distance_threshold_m < 0.0 ||
    options.angular_resolution_scale < 0.0)
  {
    return -1.0;
  }

  const double expected_angular_spacing =
    0.5 * (first_range_m + second_range_m) * std::abs(angle_increment_rad);
  const double threshold =
    options.base_distance_threshold_m +
    options.angular_resolution_scale * expected_angular_spacing;

  if (!std::isfinite(threshold) || threshold < 0.0) {
    return -1.0;
  }
  return threshold;
}

inline bool pointsAreAdjacent(
  const ScanPoint2D & first,
  const ScanPoint2D & second,
  const double angle_increment_rad,
  const ClusteringOptions & options)
{
  const double threshold = adaptiveNeighborDistance(
    first.range_m, second.range_m, angle_increment_rad, options);
  if (threshold < 0.0) {
    return false;
  }
  return planarDistance(first, second) <= threshold;
}

/// later 가 earlier 의 바로 다음 빔이고 둘이 지지 반경 안이면 서로를 지지한다.
inline bool neighborSupportsPoint(
  const ScanPoint2D & earlier,
  const ScanPoint2D & later,
  const ScanTopology & topology,
  const ClusteringOptions & options)
{
  if (!scanIndexFollows(earlier.scan_index, later.scan_index, topology)) {
    return false;
  }

  const double link_threshold = adaptiveNeighborDistance(
    earlier.range_m, later.range_m, topology.angle_increment_rad, options);
  if (link_threshold < 0.0) {
    return false;
  }

  const double threshold = options.neighbor_distance_scale * link_threshold;
  return std::isfinite(threshold) &&
         threshold >= 0.0 &&
         planarDistance(earlier, later) <= threshold;
}

/// 관심 영역 안의 빔만 직교 좌표로 바꾼다. 인덱스 오름차순.
/// 범위/FOV 밖 빔은 여기서 버려지고, 그 자리는 인덱스 불연속이 되어
/// 클러스터가 자연히 끊긴다.
inline std::vector<ScanPoint2D> extractScanPoints(
  const sensor_msgs::msg::LaserScan & scan,
  const double range_min_m,
  const double range_max_m,
  const double fov_half_rad)
{
  std::vector<ScanPoint2D> points;
  if (!hasValidScanGeometry(scan)) {
    return points;
  }

  const double lower_m = std::max(static_cast<double>(scan.range_min), range_min_m);
  const double upper_m = std::min(static_cast<double>(scan.range_max), range_max_m);
  if (!(upper_m > lower_m)) {
    return points;
  }

  points.reserve(scan.ranges.size());
  for (std::size_t index = 0U; index < scan.ranges.size(); ++index) {
    const float range = scan.ranges[index];
    if (!isFiniteFloat(range)) {
      continue;
    }

    const double range_m = static_cast<double>(range);
    if (range_m < lower_m || range_m > upper_m) {
      continue;
    }

    const double angle =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    if (!std::isfinite(angle)) {
      continue;
    }
    if (std::abs(wrapPi(angle)) > fov_half_rad) {
      continue;
    }

    points.push_back(ScanPoint2D{
        index,
        range_m * std::cos(angle),
        range_m * std::sin(angle),
        range_m,
        angle});
  }

  return points;
}

/// 이웃 지지가 없는 단발 빔을 버린다. 클러스터링과 같은 적응 임계를 쓰므로
/// 연결을 하나라도 가진 점은 (scale 1.0 에서) 절대 제거되지 않는다.
/// 설정이 쓸 수 없으면 아무것도 지우지 않는다 -- 필터 오설정이 측정값을
/// 조용히 삼키면 안 된다.
inline std::vector<ScanPoint2D> filterIsolatedScanPoints(
  const std::vector<ScanPoint2D> & points,
  const ScanTopology & topology,
  const ClusteringOptions & options)
{
  const bool filter_is_usable =
    options.speckle_filter_enabled &&
    options.minimum_support_neighbors > 0U &&
    std::isfinite(options.neighbor_distance_scale) &&
    options.neighbor_distance_scale >= 0.0 &&
    topology.valid;
  if (!filter_is_usable) {
    return points;
  }

  const bool has_wrap_pair = topology.wraps_full_circle && points.size() > 1U;
  const bool wrap_pair_supports =
    has_wrap_pair &&
    neighborSupportsPoint(points.back(), points.front(), topology, options);

  std::vector<ScanPoint2D> kept;
  kept.reserve(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    std::size_t support_count = 0U;

    if (index > 0U &&
      neighborSupportsPoint(points[index - 1U], points[index], topology, options))
    {
      ++support_count;
    }
    if (index + 1U < points.size() &&
      neighborSupportsPoint(points[index], points[index + 1U], topology, options))
    {
      ++support_count;
    }
    if (wrap_pair_supports && (index == 0U || index + 1U == points.size())) {
      ++support_count;
    }

    if (support_count >= options.minimum_support_neighbors) {
      kept.push_back(points[index]);
    }
  }

  return kept;
}

/// 인접 빔 묶기. 최소 점 수 규칙은 여기서 적용하지 않는다.
inline std::vector<Cluster2D> clusterScanPoints(
  const std::vector<ScanPoint2D> & points,
  const ScanTopology & topology,
  const ClusteringOptions & options)
{
  std::vector<Cluster2D> clusters;
  if (options.base_distance_threshold_m < 0.0 ||
    options.angular_resolution_scale < 0.0 ||
    !topology.valid ||
    points.empty())
  {
    return clusters;
  }

  for (const auto & point : points) {
    if (clusters.empty()) {
      clusters.push_back(Cluster2D{{point}, false});
      continue;
    }

    auto & current_cluster = clusters.back();
    const auto & previous_point = current_cluster.points.back();
    const bool consecutive_indices = point.scan_index == previous_point.scan_index + 1U;
    if (!consecutive_indices ||
      !pointsAreAdjacent(previous_point, point, topology.angle_increment_rad, options))
    {
      clusters.push_back(Cluster2D{{point}, false});
      continue;
    }

    current_cluster.points.push_back(point);
  }

  // 전방 FOV 가 스캔 시작 인덱스를 가로지르면 한 물체가 양 끝으로 쪼개진다.
  const bool can_merge_boundary_clusters =
    clusters.size() > 1U &&
    topology.wraps_full_circle &&
    points.front().scan_index == 0U &&
    points.back().scan_index == topology.beam_count - 1U &&
    pointsAreAdjacent(
      clusters.back().points.back(),
      clusters.front().points.front(),
      topology.angle_increment_rad,
      options);

  if (can_merge_boundary_clusters) {
    Cluster2D merged_cluster = clusters.back();
    clusters.pop_back();
    const Cluster2D first_cluster = clusters.front();
    clusters.erase(clusters.begin());
    merged_cluster.points.insert(
      merged_cluster.points.end(),
      first_cluster.points.begin(),
      first_cluster.points.end());
    merged_cluster.wraps_scan_boundary = true;
    clusters.insert(clusters.begin(), merged_cluster);
  }

  return clusters;
}

inline std::vector<Cluster2D> selectCandidateClusters(
  const std::vector<Cluster2D> & clusters,
  const std::size_t minimum_cluster_points)
{
  const std::size_t minimum_points = std::max<std::size_t>(1U, minimum_cluster_points);

  std::vector<Cluster2D> accepted;
  accepted.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    if (cluster.points.size() >= minimum_points) {
      accepted.push_back(cluster);
    }
  }
  return accepted;
}

namespace detail
{

inline double crossProduct(
  const ScanPoint2D & origin,
  const ScanPoint2D & first,
  const ScanPoint2D & second)
{
  return (first.x_m - origin.x_m) * (second.y_m - origin.y_m) -
         (first.y_m - origin.y_m) * (second.x_m - origin.x_m);
}

/// cross_product 는 m^2 (삼각형 넓이의 2 배) 이므로 선분 길이로 나누면 m 이 된다.
inline double perpendicularDistance(
  const ScanPoint2D & line_start,
  const ScanPoint2D & line_end,
  const ScanPoint2D & point)
{
  const double line_length = planarDistance(line_start, line_end);
  if (line_length <= 0.0) {
    return planarDistance(line_start, point);
  }
  return std::abs(crossProduct(line_start, line_end, point)) / line_length;
}

/// monotone chain 한 단계. 좌회전이 아니면 가운데 점을 버린다. 여기서는
/// 허용오차를 쓰지 않아 껍질이 모든 입력점을 반드시 포함한다.
inline void appendHullChain(
  const std::vector<ScanPoint2D> & sorted_points,
  const bool reversed,
  std::vector<ScanPoint2D> & chain)
{
  const std::size_t count = sorted_points.size();
  for (std::size_t step = 0U; step < count; ++step) {
    const auto & point = reversed ? sorted_points[count - 1U - step] : sorted_points[step];
    while (chain.size() >= 2U &&
      crossProduct(chain[chain.size() - 2U], chain.back(), point) <= 0.0)
    {
      chain.pop_back();
    }
    chain.push_back(point);
  }
}

inline std::vector<ScanPoint2D> sortedUniquePoints(const std::vector<ScanPoint2D> & points)
{
  std::vector<ScanPoint2D> sorted = points;
  std::sort(
    sorted.begin(), sorted.end(),
    [](const ScanPoint2D & first, const ScanPoint2D & second) {
      if (first.x_m != second.x_m) {
        return first.x_m < second.x_m;
      }
      if (first.y_m != second.y_m) {
        return first.y_m < second.y_m;
      }
      return first.scan_index < second.scan_index;
    });
  sorted.erase(
    std::unique(
      sorted.begin(), sorted.end(),
      [](const ScanPoint2D & first, const ScanPoint2D & second) {
        return first.x_m == second.x_m && first.y_m == second.y_m;
      }),
    sorted.end());
  return sorted;
}

/// 가장 먼 두 점. 극점 쌍은 항상 볼록 껍질 위에 있다.
inline std::pair<std::size_t, std::size_t> extremeVertexPair(
  const std::vector<ScanPoint2D> & vertices)
{
  std::pair<std::size_t, std::size_t> extremes{0U, 0U};
  double largest = 0.0;
  for (std::size_t first = 0U; first < vertices.size(); ++first) {
    for (std::size_t second = first + 1U; second < vertices.size(); ++second) {
      const double distance = planarDistance(vertices[first], vertices[second]);
      if (distance > largest) {
        largest = distance;
        extremes = {first, second};
      }
    }
  }
  return extremes;
}

/// 모든 껍질 점이 극점 쌍을 잇는 선에서 collinear_tolerance_m 안이면 두 점으로
/// 접는다. 판단이 전역이라 접힌 껍질도 관측점을 허용오차 밖에 남기지 않는다.
inline std::vector<ScanPoint2D> collapseCollinearHull(
  const std::vector<ScanPoint2D> & hull,
  const double collinear_tolerance_m)
{
  if (hull.size() < 3U) {
    return hull;
  }

  const auto extremes = extremeVertexPair(hull);
  const auto & line_start = hull[extremes.first];
  const auto & line_end = hull[extremes.second];
  for (const auto & vertex : hull) {
    if (perpendicularDistance(line_start, line_end, vertex) > collinear_tolerance_m) {
      return hull;
    }
  }
  return {line_start, line_end};
}

}  // namespace detail

/// 관측점의 볼록 껍질. 안전 여유나 미관측면 보완이 들어가지 않는다.
inline std::vector<ScanPoint2D> computeObservedFootprint(
  const std::vector<ScanPoint2D> & points,
  const double collinear_tolerance_m)
{
  std::vector<ScanPoint2D> footprint;
  if (points.empty() || !std::isfinite(collinear_tolerance_m) || collinear_tolerance_m < 0.0) {
    return footprint;
  }
  if (!std::all_of(points.begin(), points.end(), pointIsFinite)) {
    return footprint;
  }

  const auto unique_points = detail::sortedUniquePoints(points);
  if (unique_points.size() < 3U) {
    return unique_points;
  }

  std::vector<ScanPoint2D> lower_chain;
  detail::appendHullChain(unique_points, false, lower_chain);
  std::vector<ScanPoint2D> upper_chain;
  detail::appendHullChain(unique_points, true, upper_chain);

  footprint.reserve(lower_chain.size() + upper_chain.size());
  footprint.insert(footprint.end(), lower_chain.begin(), lower_chain.end() - 1);
  footprint.insert(footprint.end(), upper_chain.begin(), upper_chain.end() - 1);
  if (footprint.size() < 3U) {
    return {unique_points.front(), unique_points.back()};
  }
  return detail::collapseCollinearHull(footprint, collinear_tolerance_m);
}

inline ClusterGeometry2D computeClusterGeometry(
  const Cluster2D & cluster,
  const double collinear_tolerance_m)
{
  ClusterGeometry2D geometry;
  if (cluster.points.empty() ||
    !std::isfinite(collinear_tolerance_m) ||
    collinear_tolerance_m < 0.0)
  {
    return geometry;
  }
  if (!std::all_of(cluster.points.begin(), cluster.points.end(), pointIsFinite)) {
    return geometry;
  }

  const auto & points = cluster.points;
  const auto closest = std::min_element(
    points.begin(), points.end(),
    [](const ScanPoint2D & first, const ScanPoint2D & second) {
      if (first.range_m != second.range_m) {
        return first.range_m < second.range_m;
      }
      return first.scan_index < second.scan_index;
    });

  const auto footprint = computeObservedFootprint(points, collinear_tolerance_m);
  double max_extent_m = 0.0;
  if (footprint.size() >= 2U) {
    const auto extremes = detail::extremeVertexPair(footprint);
    max_extent_m = planarDistance(footprint[extremes.first], footprint[extremes.second]);
  }

  geometry.closest_point = *closest;
  geometry.width_m = planarDistance(points.front(), points.back());
  geometry.max_extent_m = max_extent_m;
  geometry.point_count = points.size();
  geometry.valid = true;
  return geometry;
}

/// 폭 게이트가 쓰는 관측 크기. 현 길이와 껍질 최대 거리 중 큰 쪽이다.
inline double observedExtent(const ClusterGeometry2D & geometry)
{
  const double width = std::isfinite(geometry.width_m) ? geometry.width_m : 0.0;
  const double extent = std::isfinite(geometry.max_extent_m) ? geometry.max_extent_m : 0.0;
  return std::max(width, extent);
}

inline bool coneOptionsAreUsable(const ConeCircleOptions & options)
{
  return std::isfinite(options.nominal_cone_radius_m) &&
         options.nominal_cone_radius_m > 0.0 &&
         std::isfinite(options.visible_slice_radius_m) &&
         options.visible_slice_radius_m >= 0.0 &&
         std::isfinite(options.maximum_observed_width_m) &&
         options.maximum_observed_width_m > 0.0;
}

/// 관측 클러스터 하나에서 콘 축 중심을 복원한다.
///
/// 라이다는 콘의 앞면만 본다. 최근접점을 그 방위 그대로 센서 반대쪽으로
/// visible_slice_radius_m 만큼 밀면 축 중심이 나온다. centroid 를 그대로
/// 쓰면 관측면 쪽으로 치우친다.
inline ConeCircle computeConeCircle(
  const ClusterGeometry2D & geometry,
  const ConeCircleOptions & options)
{
  ConeCircle circle;
  if (!coneOptionsAreUsable(options) || !geometry.valid || geometry.point_count == 0U) {
    return circle;
  }
  if (!std::isfinite(geometry.width_m) || !std::isfinite(geometry.max_extent_m)) {
    return circle;
  }

  const auto & closest = geometry.closest_point;
  const double closest_norm_m = std::hypot(closest.x_m, closest.y_m);
  if (!pointIsFinite(closest) ||
    closest.range_m <= 0.0 ||
    !std::isfinite(closest_norm_m) ||
    closest_norm_m <= 0.0)
  {
    return circle;
  }

  const double extent_m = observedExtent(geometry);
  if (extent_m > options.maximum_observed_width_m) {
    return circle;
  }

  const double direction_x = closest.x_m / closest_norm_m;
  const double direction_y = closest.y_m / closest_norm_m;

  circle.center_x_m = closest.x_m + direction_x * options.visible_slice_radius_m;
  circle.center_y_m = closest.y_m + direction_y * options.visible_slice_radius_m;
  circle.radius_m = options.nominal_cone_radius_m;
  circle.observed_extent_m = extent_m;
  circle.point_count = geometry.point_count;

  if (!std::isfinite(circle.center_x_m) || !std::isfinite(circle.center_y_m)) {
    return ConeCircle{};
  }

  circle.valid = true;
  return circle;
}

}  // namespace backup_object_detection

#endif  // BACKUP_OBJECT_DETECTION__CLUSTERING_HPP_
