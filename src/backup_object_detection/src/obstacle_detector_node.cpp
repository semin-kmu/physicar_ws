#include "backup_object_detection/obstacle_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace backup_object_detection
{
namespace
{

constexpr double kDeg2Rad = kPi / 180.0;

/// 연결 게이트를 통과한 트랙-측정 쌍 하나.
struct AssociationPair
{
  double distance_m{0.0};
  std::size_t track_index{0U};
  std::size_t measurement_index{0U};
};

bool measurementIsFinite(const TrackedObstacle & measurement)
{
  return std::isfinite(measurement.x_m) &&
         std::isfinite(measurement.y_m) &&
         std::isfinite(measurement.radius_m);
}

/// 잘못된 값이 와도 추적기가 예측 가능하게 돈다. yaml 오타로 주행 중
/// 노드가 죽는 것보다 clamp 가 낫다 (검증은 노드 생성자가 이미 했다).
ObstacleTrackOptions clampTrackOptions(const ObstacleTrackOptions & options)
{
  ObstacleTrackOptions clamped = options;

  if (!std::isfinite(clamped.association_distance_m) || clamped.association_distance_m < 0.0) {
    clamped.association_distance_m = 0.0;
  }
  if (!std::isfinite(clamped.outlier_distance_m) || clamped.outlier_distance_m < 0.0) {
    clamped.outlier_distance_m = 0.0;
  }
  if (!std::isfinite(clamped.smoothing_alpha)) {
    clamped.smoothing_alpha = 1.0;
  }
  clamped.smoothing_alpha = std::min(std::max(clamped.smoothing_alpha, 0.0), 1.0);
  if (clamped.minimum_confirmation_frames < 1U) {
    clamped.minimum_confirmation_frames = 1U;
  }
  if (!std::isfinite(clamped.track_timeout_s) || clamped.track_timeout_s < 0.0) {
    clamped.track_timeout_s = 0.0;
  }

  return clamped;
}

}  // namespace

ObstacleTracker::ObstacleTracker(const ObstacleTrackOptions & options)
: options_(clampTrackOptions(options))
{
}

void ObstacleTracker::reset()
{
  tracks_.clear();
  has_timestamp_ = false;
  last_timestamp_s_ = 0.0;
}

std::vector<TrackedObstacle> ObstacleTracker::update(
  const std::vector<TrackedObstacle> & measurements,
  const double timestamp_s)
{
  std::vector<TrackedObstacle> confirmed;

  // 타임라인에 올릴 수 없는 프레임은 타임아웃이 의존하는 나이를 전부
  // 무의미하게 만든다. 임의로 오래된 트랙을 남기느니 처음부터 다시 센다.
  if (!std::isfinite(timestamp_s) || (has_timestamp_ && timestamp_s < last_timestamp_s_)) {
    reset();
  }
  if (!std::isfinite(timestamp_s)) {
    return confirmed;
  }

  // 타임아웃 제거가 연결보다 먼저 돈다. 이미 만료된 트랙에 측정이 붙는 일이 없다.
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this, timestamp_s](const Track & track) {
        return (timestamp_s - track.last_observed_s) > options_.track_timeout_s;
      }),
    tracks_.end());

  for (auto & track : tracks_) {
    track.observed_this_frame = false;
  }

  std::vector<bool> measurement_used(measurements.size(), false);
  for (std::size_t index = 0U; index < measurements.size(); ++index) {
    if (!measurementIsFinite(measurements[index])) {
      measurement_used[index] = true;
    }
  }

  // 결정적 greedy 1:1 연결. 가까운 쌍부터 소비하고, 동률은 트랙 id 와
  // 측정 인덱스로 깨므로 결과가 반복 순서에 의존하지 않는다.
  std::vector<AssociationPair> pairs;
  pairs.reserve(tracks_.size() * measurements.size());
  for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
    for (std::size_t index = 0U; index < measurements.size(); ++index) {
      if (measurement_used[index]) {
        continue;
      }
      const double distance = std::hypot(
        tracks_[track_index].x_m - measurements[index].x_m,
        tracks_[track_index].y_m - measurements[index].y_m);
      if (distance > options_.association_distance_m) {
        continue;
      }
      pairs.push_back(AssociationPair{distance, track_index, index});
    }
  }

  std::sort(
    pairs.begin(), pairs.end(),
    [this](const AssociationPair & first, const AssociationPair & second) {
      if (first.distance_m != second.distance_m) {
        return first.distance_m < second.distance_m;
      }
      if (tracks_[first.track_index].id != tracks_[second.track_index].id) {
        return tracks_[first.track_index].id < tracks_[second.track_index].id;
      }
      return first.measurement_index < second.measurement_index;
    });

  std::vector<bool> track_linked(tracks_.size(), false);
  for (const auto & pair : pairs) {
    if (track_linked[pair.track_index] || measurement_used[pair.measurement_index]) {
      continue;
    }
    track_linked[pair.track_index] = true;
    measurement_used[pair.measurement_index] = true;

    // 이상치 게이트. 연결은 어느 트랙인지만 정하고, 그 측정이 추정을 움직일
    // 만큼 믿을 만한지는 여기서 따로 판단한다.
    if (pair.distance_m > options_.outlier_distance_m) {
      continue;
    }

    auto & track = tracks_[pair.track_index];
    const auto & measurement = measurements[pair.measurement_index];
    const double alpha = options_.smoothing_alpha;
    track.x_m = (alpha * measurement.x_m) + ((1.0 - alpha) * track.x_m);
    track.y_m = (alpha * measurement.y_m) + ((1.0 - alpha) * track.y_m);
    // 반지름은 설정값이라 평활하지 않고 그대로 싣는다.
    track.radius_m = measurement.radius_m;
    track.last_observed_s = timestamp_s;
    track.observed_this_frame = true;
    ++track.observation_count;
  }

  for (std::size_t index = 0U; index < measurements.size(); ++index) {
    if (measurement_used[index]) {
      continue;
    }
    Track track;
    track.id = next_track_id_++;
    track.x_m = measurements[index].x_m;
    track.y_m = measurements[index].y_m;
    track.radius_m = measurements[index].radius_m;
    track.observation_count = 1U;
    track.last_observed_s = timestamp_s;
    track.observed_this_frame = true;
    tracks_.push_back(track);
  }

  std::sort(
    tracks_.begin(), tracks_.end(),
    [](const Track & first, const Track & second) {return first.id < second.id;});

  confirmed.reserve(tracks_.size());
  for (const auto & track : tracks_) {
    // 두 조건 모두 필요하다. 이번 프레임에 관측되지 않은 트랙은 예측값이므로
    // 발행하지 않고, 갓 생긴 트랙은 확인 전까지 내부에만 둔다.
    if (!track.observed_this_frame ||
      track.observation_count < options_.minimum_confirmation_frames)
    {
      continue;
    }
    confirmed.push_back(TrackedObstacle{track.x_m, track.y_m, track.radius_m});
  }

  last_timestamp_s_ = timestamp_s;
  has_timestamp_ = true;
  return confirmed;
}

ObstacleDetectorNode::ObstacleDetectorNode()
: Node("backup_obstacle_detector")
{
  const auto input_topic = declare_parameter<std::string>("input_topic", "/scan_filtered");
  const auto output_topic =
    declare_parameter<std::string>("output_topic", "/backup/perception/obstacles");

  expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "lidar_link");
  const auto minimum_sample_count = declare_parameter<std::int64_t>("minimum_sample_count", 2);
  const auto sample_count_tolerance = declare_parameter<std::int64_t>("sample_count_tolerance", 1);
  require_intensities_ = declare_parameter<bool>("require_intensities", false);

  lidar_x_m_ = declare_parameter<double>("lidar_x_m", -0.027);
  lidar_y_m_ = declare_parameter<double>("lidar_y_m", 0.0);
  const auto lidar_yaw_rad = declare_parameter<double>("lidar_yaw_rad", 0.0);
  rear_axle_offset_m_ = declare_parameter<double>("rear_axle_offset_m", -0.090);

  clustering_.speckle_filter_enabled = declare_parameter<bool>("speckle_filter_enabled", true);
  const auto minimum_support_neighbors =
    declare_parameter<std::int64_t>("minimum_support_neighbors", 1);
  clustering_.neighbor_distance_scale = declare_parameter<double>("neighbor_distance_scale", 1.0);

  clustering_.base_distance_threshold_m =
    declare_parameter<double>("base_distance_threshold_m", 0.05);
  clustering_.angular_resolution_scale =
    declare_parameter<double>("angular_resolution_scale", 1.5);
  const auto minimum_cluster_points = declare_parameter<std::int64_t>("minimum_cluster_points", 1);
  clustering_.circular_scan_tolerance_rad =
    declare_parameter<double>("circular_scan_tolerance_rad", 0.05);
  clustering_.collinear_tolerance_m = declare_parameter<double>("collinear_tolerance_m", 0.001);

  cone_.nominal_cone_radius_m = declare_parameter<double>("nominal_cone_radius_m", 0.09);
  cone_.visible_slice_radius_m = declare_parameter<double>("visible_slice_radius_m", 0.043);
  cone_.maximum_observed_width_m = declare_parameter<double>("maximum_observed_width_m", 0.3);

  obstacle_radius_m_ = declare_parameter<double>("obstacle_radius_m", 0.142);
  use_fixed_radius_ = declare_parameter<bool>("use_fixed_radius", true);

  temporal_tracking_enabled_ = declare_parameter<bool>("temporal_tracking_enabled", true);
  ObstacleTrackOptions track_options;
  const auto minimum_confirmation_frames =
    declare_parameter<std::int64_t>("minimum_confirmation_frames", 2);
  track_options.association_distance_m = declare_parameter<double>("association_distance_m", 0.3);
  track_options.outlier_distance_m = declare_parameter<double>("outlier_distance_m", 0.2);
  track_options.track_timeout_s = declare_parameter<double>("track_timeout_s", 0.7);
  track_options.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 0.2);

  range_min_m_ = declare_parameter<double>("range_min_m", 0.10);
  range_max_m_ = declare_parameter<double>("range_max_m", 2.50);
  const auto fov_half_deg = declare_parameter<double>("fov_half_deg", 90.0);
  confidence_ = declare_parameter<double>("confidence", 1.0);
  const auto log_period_s = declare_parameter<double>("log_period_s", 2.0);

  if (minimum_sample_count < 2 || minimum_sample_count > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("minimum_sample_count must be at least 2 and fit in an int");
  }
  if (sample_count_tolerance < 0 || sample_count_tolerance > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("sample_count_tolerance must be non-negative and fit in an int");
  }
  if (minimum_cluster_points < 1 || minimum_cluster_points > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("minimum_cluster_points must be at least 1 and fit in an int");
  }
  if (minimum_support_neighbors < 0 ||
    minimum_support_neighbors > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("minimum_support_neighbors must be non-negative and fit in an int");
  }
  if (minimum_confirmation_frames < 1 ||
    minimum_confirmation_frames > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument("minimum_confirmation_frames must be at least 1 and fit in an int");
  }
  if (!coneOptionsAreUsable(cone_)) {
    throw std::invalid_argument(
            "nominal_cone_radius_m, visible_slice_radius_m and maximum_observed_width_m "
            "do not describe a usable cone model");
  }
  if (!std::isfinite(obstacle_radius_m_) || obstacle_radius_m_ <= 0.0) {
    throw std::invalid_argument("obstacle_radius_m must be finite and positive");
  }
  if (!std::isfinite(range_min_m_) || !std::isfinite(range_max_m_) ||
    range_min_m_ < 0.0 || range_max_m_ <= range_min_m_)
  {
    throw std::invalid_argument("range_min_m and range_max_m do not define a valid interval");
  }
  if (!std::isfinite(fov_half_deg) || fov_half_deg <= 0.0 || fov_half_deg > 180.0) {
    throw std::invalid_argument("fov_half_deg must be within (0, 180]");
  }
  if (!std::isfinite(confidence_) || confidence_ < 0.0 || confidence_ > 1.0) {
    throw std::invalid_argument("confidence must be within 0..1");
  }
  if (!std::isfinite(log_period_s) || log_period_s <= 0.0) {
    throw std::invalid_argument("log_period_s must be finite and positive");
  }
  if (!std::isfinite(lidar_yaw_rad) || !std::isfinite(rear_axle_offset_m_) ||
    !std::isfinite(lidar_x_m_) || !std::isfinite(lidar_y_m_))
  {
    throw std::invalid_argument("lidar mounting parameters must be finite");
  }

  minimum_sample_count_ = static_cast<std::size_t>(minimum_sample_count);
  sample_count_tolerance_ = static_cast<std::size_t>(sample_count_tolerance);
  clustering_.minimum_cluster_points = static_cast<std::size_t>(minimum_cluster_points);
  clustering_.minimum_support_neighbors = static_cast<std::size_t>(minimum_support_neighbors);
  track_options.minimum_confirmation_frames =
    static_cast<std::size_t>(minimum_confirmation_frames);
  lidar_cos_yaw_ = std::cos(lidar_yaw_rad);
  lidar_sin_yaw_ = std::sin(lidar_yaw_rad);
  fov_half_rad_ = fov_half_deg * kDeg2Rad;
  log_period_ms_ = static_cast<std::int64_t>(log_period_s * 1000.0);

  tracker_ = std::make_unique<ObstacleTracker>(track_options);

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::onScan, this, std::placeholders::_1));

  obstacle_publisher_ = create_publisher<backup_msgs::msg::ObstacleCircleArray>(
    output_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());

  RCLCPP_INFO(
    get_logger(),
    "[기동][obstacle] in=%s out=%s frame_out=base_link 범위 %.2f~%.2f m FOV +-%.1f deg "
    "최소점 %ld 반지름 %.3f m(고정=%d)",
    input_topic.c_str(), output_topic.c_str(), range_min_m_, range_max_m_, fov_half_deg,
    static_cast<long>(minimum_cluster_points), obstacle_radius_m_, use_fixed_radius_ ? 1 : 0);
  RCLCPP_INFO(
    get_logger(),
    "[기동][obstacle] 정적 변환 lidar(%.3f, %.3f, %.4f rad) 후륜축 보정 %.3f m, TF 조회 없음",
    lidar_x_m_, lidar_y_m_, lidar_yaw_rad, rear_axle_offset_m_);
  RCLCPP_INFO(
    get_logger(),
    "[기동][obstacle] 스페클 %d (지지 %ld, scale %.2f) 링크 %.3f m + %.2f x 빔간격, "
    "collinear %.4f m",
    clustering_.speckle_filter_enabled ? 1 : 0,
    static_cast<long>(minimum_support_neighbors), clustering_.neighbor_distance_scale,
    clustering_.base_distance_threshold_m, clustering_.angular_resolution_scale,
    clustering_.collinear_tolerance_m);
  RCLCPP_INFO(
    get_logger(),
    "[기동][obstacle] 콘 복원 nominal %.3f m, 가시면 %.3f m, 폭 상한 %.2f m",
    cone_.nominal_cone_radius_m, cone_.visible_slice_radius_m, cone_.maximum_observed_width_m);
  RCLCPP_INFO(
    get_logger(),
    "[기동][obstacle] 시간 추적 %d 확인 %ld 프레임, 연결 %.2f m, 이상치 %.2f m, "
    "alpha %.2f, timeout %.2f s",
    temporal_tracking_enabled_ ? 1 : 0, static_cast<long>(minimum_confirmation_frames),
    track_options.association_distance_m, track_options.outlier_distance_m,
    track_options.smoothing_alpha, track_options.track_timeout_s);
}

std::string ObstacleDetectorNode::validateScan(const sensor_msgs::msg::LaserScan & scan) const
{
  if (scan.header.frame_id.empty()) {
    return "frame_id 가 비어 있다";
  }
  if (!expected_frame_id_.empty() && scan.header.frame_id != expected_frame_id_) {
    return "frame_id 불일치: " + scan.header.frame_id;
  }
  if (scan.ranges.size() < minimum_sample_count_) {
    return "빔 수 부족";
  }
  if (require_intensities_ && scan.intensities.size() != scan.ranges.size()) {
    return "intensities 길이 불일치";
  }
  if (!scan.intensities.empty() && scan.intensities.size() != scan.ranges.size()) {
    return "intensities 길이 불일치";
  }
  if (!hasValidScanGeometry(scan)) {
    return "angle/range 메타데이터 불량";
  }

  // 각도 메타데이터로 유도한 빔 수와 실제 개수가 어긋나면 각도 계산이 통째로
  // 틀어진다.
  const double interval_count =
    static_cast<double>(scan.angle_max - scan.angle_min) /
    static_cast<double>(scan.angle_increment);
  if (!std::isfinite(interval_count) || interval_count < 0.0) {
    return "angle 메타데이터로 빔 수를 유도할 수 없다";
  }
  const auto expected_sample_count =
    static_cast<std::size_t>(std::llround(interval_count)) + 1U;
  const auto count_difference =
    expected_sample_count > scan.ranges.size() ?
    expected_sample_count - scan.ranges.size() :
    scan.ranges.size() - expected_sample_count;
  if (count_difference > sample_count_tolerance_) {
    return "빔 수가 angle 메타데이터와 불일치";
  }

  if (!isFiniteFloat(scan.scan_time) || !isFiniteFloat(scan.time_increment) ||
    scan.scan_time < 0.0F || scan.time_increment < 0.0F)
  {
    return "scan 타이밍 메타데이터 불량";
  }
  return std::string();
}

void ObstacleDetectorNode::toRearAxle(
  const double x_lidar_m,
  const double y_lidar_m,
  double & x_out_m,
  double & y_out_m) const
{
  // rear_axle_offset_m 는 base_footprint 기준 후륜축의 위치(-0.090)다.
  // 원점을 뒤로 옮기면 점의 x 는 그만큼 커지므로 더하지 않고 뺀다.
  x_out_m = lidar_cos_yaw_ * x_lidar_m - lidar_sin_yaw_ * y_lidar_m +
    lidar_x_m_ - rear_axle_offset_m_;
  y_out_m = lidar_sin_yaw_ * x_lidar_m + lidar_cos_yaw_ * y_lidar_m + lidar_y_m_;
}

void ObstacleDetectorNode::publishFailure(
  const builtin_interfaces::msg::Time & stamp,
  const std::uint8_t status)
{
  // 실패 프레임에서 이전 주기를 재사용하지 않는다. 트랙 상태까지 버려야
  // 복구 직후 첫 프레임이 낡은 추정을 확인된 트랙으로 되살리지 않는다.
  tracker_->reset();

  backup_msgs::msg::ObstacleCircleArray message;
  message.header.stamp = stamp;
  message.header.frame_id = "base_link";
  message.status = status;
  obstacle_publisher_->publish(message);
}

void ObstacleDetectorNode::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  try {
    const auto reason = validateScan(*scan);
    if (!reason.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_period_ms_,
        "[검출][obstacle] LaserScan 검증 실패 (%s), 빈 배열 발행", reason.c_str());
      publishFailure(
        scan->header.stamp, backup_msgs::msg::ObstacleCircleArray::STATUS_LIDAR_UNAVAILABLE);
      return;
    }

    const auto topology = makeScanTopology(*scan, clustering_.circular_scan_tolerance_rad);
    if (!topology.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), log_period_ms_,
        "[검출][obstacle] 빔 인접 구조 계산 실패, 빈 배열 발행");
      publishFailure(
        scan->header.stamp, backup_msgs::msg::ObstacleCircleArray::STATUS_LIDAR_UNAVAILABLE);
      return;
    }

    const auto points = extractScanPoints(*scan, range_min_m_, range_max_m_, fov_half_rad_);
    const auto supported = filterIsolatedScanPoints(points, topology, clustering_);
    const auto clusters = clusterScanPoints(supported, topology, clustering_);
    const auto accepted = selectCandidateClusters(clusters, clustering_.minimum_cluster_points);

    std::vector<TrackedObstacle> measurements;
    measurements.reserve(accepted.size());
    std::size_t width_rejected = 0U;
    for (const auto & cluster : accepted) {
      const auto geometry = computeClusterGeometry(cluster, clustering_.collinear_tolerance_m);
      const auto circle = computeConeCircle(geometry, cone_);
      if (!circle.valid) {
        ++width_rejected;
        continue;
      }

      TrackedObstacle measurement;
      toRearAxle(circle.center_x_m, circle.center_y_m, measurement.x_m, measurement.y_m);
      measurement.radius_m = use_fixed_radius_ ? obstacle_radius_m_ : circle.radius_m;
      measurements.push_back(measurement);
    }

    // 추적기 시각은 this->now() 다. 센서 stamp 는 드라이버 시계에 따라 뒤로
    // 뛸 수 있고, 그러면 추적기가 매 프레임 초기화된다.
    const auto published = temporal_tracking_enabled_ ?
      tracker_->update(measurements, now().seconds()) :
      measurements;

    backup_msgs::msg::ObstacleCircleArray message;
    message.header.stamp = scan->header.stamp;
    message.header.frame_id = "base_link";
    message.status = backup_msgs::msg::ObstacleCircleArray::STATUS_OK;
    message.obstacles.reserve(published.size());
    for (const auto & obstacle : published) {
      backup_msgs::msg::ObstacleCircle circle;
      circle.center_x = static_cast<float>(obstacle.x_m);
      circle.center_y = static_cast<float>(obstacle.y_m);
      circle.radius = static_cast<float>(obstacle.radius_m);
      circle.confidence = static_cast<float>(confidence_);
      message.obstacles.push_back(circle);
    }

    obstacle_publisher_->publish(message);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), log_period_ms_,
      "[검출][obstacle] 발행 %zu 개 (빔 %zu->%zu, 클러스터 %zu, 후보 %zu, 폭초과 %zu, "
      "트랙 %zu)",
      message.obstacles.size(), points.size(), supported.size(), accepted.size(),
      measurements.size(), width_rejected, tracker_->activeTrackCount());
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), log_period_ms_,
      "[검출][obstacle] 콜백 예외 (%s), 빈 배열 발행", error.what());
    publishFailure(
      scan->header.stamp, backup_msgs::msg::ObstacleCircleArray::STATUS_INTERNAL_ERROR);
  }
}

}  // namespace backup_object_detection
