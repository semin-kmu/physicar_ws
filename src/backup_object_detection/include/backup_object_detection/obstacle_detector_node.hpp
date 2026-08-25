// backup_obstacle_detector
//
// /scan_filtered 콜백 구동 (~9.8 Hz). 타이머 없다.
// 빔 -> 스페클 제거 -> 클러스터 -> 콘 중심 복원 -> 시간 추적 ->
// 고정 반지름 원 -> /backup/perception/obstacles.
// TF 를 조회하지 않는다. 측위 비의존이 백업 스택의 전제다.

#ifndef BACKUP_OBJECT_DETECTION__OBSTACLE_DETECTOR_NODE_HPP_
#define BACKUP_OBJECT_DETECTION__OBSTACLE_DETECTOR_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "backup_msgs/msg/obstacle_circle_array.hpp"
#include "backup_object_detection/clustering.hpp"

namespace backup_object_detection
{

struct ObstacleTrackOptions
{
  /// 이 거리 안이면 같은 장애물로 연결한다. 가장 가까운 두 콘 간격보다
  /// 충분히 작아야 두 장애물이 한 트랙으로 뭉치지 않는다.
  double association_distance_m{0.30};
  /// 연결된 측정이 이전 추정에서 이만큼 넘게 떨어져 있으면 순간 이상치로
  /// 보고 갱신에 넣지 않는다. 연결 게이트와 역할이 다르다.
  double outlier_distance_m{0.20};
  /// estimate = alpha * measurement + (1 - alpha) * previous.
  double smoothing_alpha{0.2};
  /// 발행 자격을 얻기까지 필요한 관측 수. 1 프레임 유령을 걸러낸다.
  std::size_t minimum_confirmation_frames{2U};
  /// 관측 없이 트랙이 버티는 시간. 내부 저장소만 지배하며, 이번 프레임에
  /// 관측되지 않은 트랙은 어차피 발행되지 않는다.
  double track_timeout_s{0.7};
};

/// 트랙 입력이자 출력. 반지름은 추정하지 않고 그대로 실어 나른다.
struct TrackedObstacle
{
  double x_m{0.0};
  double y_m{0.0};
  double radius_m{0.0};
};

/// 시간 추적기. kau_object_detection/obstacle_tracker 이식.
/// ROS 타입을 쓰지 않는다 -- 시각은 호출자가 초 단위로 준다.
///
/// 한 프레임: 타임아웃 제거 -> 결정적 greedy 1:1 연결 -> 이상치 게이트 ->
/// EMA 갱신 -> 미연결 측정으로 신규 트랙 생성 -> 확인된 트랙만 반환.
class ObstacleTracker
{
public:
  explicit ObstacleTracker(const ObstacleTrackOptions & options);

  std::vector<TrackedObstacle> update(
    const std::vector<TrackedObstacle> & measurements,
    double timestamp_s);

  void reset();

  std::size_t activeTrackCount() const {return tracks_.size();}

private:
  struct Track
  {
    std::uint64_t id{0U};
    double x_m{0.0};
    double y_m{0.0};
    double radius_m{0.0};
    std::size_t observation_count{0U};
    double last_observed_s{0.0};
    bool observed_this_frame{false};
  };

  ObstacleTrackOptions options_;
  std::vector<Track> tracks_;
  std::uint64_t next_track_id_{1U};
  double last_timestamp_s_{0.0};
  bool has_timestamp_{false};
};

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan);

  /// 구조 검증. 실패 사유를 반환하고, 통과하면 빈 문자열.
  std::string validateScan(const sensor_msgs::msg::LaserScan & scan) const;

  /// non-OK 는 항상 빈 배열이다. 이전 주기 재사용 금지가 시간 추적보다
  /// 우선하므로 트랙 상태도 함께 버린다.
  void publishFailure(const builtin_interfaces::msg::Time & stamp, std::uint8_t status);

  /// lidar_link -> base_footprint 정적 변환 후 후륜축 원점으로 평행이동.
  void toRearAxle(double x_lidar_m, double y_lidar_m, double & x_out_m, double & y_out_m) const;

  std::string expected_frame_id_;
  std::size_t minimum_sample_count_{2U};
  std::size_t sample_count_tolerance_{1U};
  bool require_intensities_{false};

  double lidar_x_m_{-0.027};
  double lidar_y_m_{0.0};
  double lidar_cos_yaw_{1.0};
  double lidar_sin_yaw_{0.0};
  double rear_axle_offset_m_{-0.090};

  ClusteringOptions clustering_;
  ConeCircleOptions cone_;
  double obstacle_radius_m_{0.142};
  bool use_fixed_radius_{true};

  bool temporal_tracking_enabled_{true};
  std::unique_ptr<ObstacleTracker> tracker_;

  double range_min_m_{0.10};
  double range_max_m_{2.50};
  double fov_half_rad_{kPi / 2.0};
  double confidence_{1.0};

  std::int64_t log_period_ms_{2000};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<backup_msgs::msg::ObstacleCircleArray>::SharedPtr obstacle_publisher_;
};

}  // namespace backup_object_detection

#endif  // BACKUP_OBJECT_DETECTION__OBSTACLE_DETECTOR_NODE_HPP_
