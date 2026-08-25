// 지면 z=0 호모그래피와 행별 사전계산 테이블.
//
// 전체 warpPerspective 를 하지 않는다. 3x3 H 를 기동 시 1회 만들고
// 검출점(수백 개)에만 건다 -- warp 14.8 ms 대 ~0.
// 카메라가 고정이므로 행별 거리/스케일/선폭도 기동 시 1회다.

#ifndef BACKUP_LANE_DETECTION__HOMOGRAPHY_HPP_
#define BACKUP_LANE_DETECTION__HOMOGRAPHY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

namespace backup_lane_detection
{

// 카메라 프레임 x 우 / y 하 / z 전방, 하향틸트 a.
// 광선 dx=(u-cx)/fx, dy=(v-cy)/fy 에 대해
//   W = dy*cos a + sin a            W <= 0 이면 지평선 위 -- 지면과 안 만난다
//   X = h*(cos a - dy*sin a) / W    [m] 카메라 접지점 기준 전방
//   Y = -h*dx / W                   [m] 좌측 +
// 위 셋이 (u,v,1) 에 선형이므로 3x3 로 접는다.
struct GroundProjector
{
  cv::Matx33d H = cv::Matx33d::eye();   // (u,v,1) -> (X*W, Y*W, W)
  double horizon_row = 0.0;

  void build(double fx, double fy, double cx, double cy, double h, double tilt_rad)
  {
    const double ca = std::cos(tilt_rad);
    const double sa = std::sin(tilt_rad);

    H = cv::Matx33d(
      0.0, -h * sa / fy, h * ca + h * sa * cy / fy,
      -h / fx, 0.0, h * cx / fx,
      0.0, ca / fy, sa - ca * cy / fy);

    horizon_row = cy - fy * std::tan(tilt_rad);
  }

  // 지면과 만나지 않으면 false.
  bool project(double u, double v, double & x, double & y) const
  {
    const double w = H(2, 0) * u + H(2, 1) * v + H(2, 2);
    if (w <= 1e-9) {return false;}
    x = (H(0, 0) * u + H(0, 1) * v + H(0, 2)) / w;
    y = (H(1, 0) * u + H(1, 1) * v + H(1, 2)) / w;
    return true;
  }
};

// 한 행의 기하. 사전계산 테이블의 원소.
struct RowGeometry
{
  bool valid = false;
  double x_rear = 0.0;       // [m] 후륜축 원점 종방향
  double lat_scale = 0.0;    // [m/px] 횡
  double lane_px_row = 0.0;  // 행 스캔 기대 런 폭 [px]
  double lane_px_col = 0.0;  // 열 스캔 기대 런 폭 [px]
};

// 규약 검산식.
//   d = h / tan( atan((v-cy)/fy) + a ),  x = d + rear_offset
//   횡스케일 = hypot(h, d) / fx,  선폭 px = lane_width_m / 횡스케일
// 종스케일은 dX/dv 를 그대로 미분한 h / (fy * W^2) 다 (열 런 폭에만 쓴다).
inline RowGeometry rowGeometry(
  double v, double fx, double fy, double cy, double h, double tilt_rad,
  double rear_offset, double lane_width_m)
{
  RowGeometry g;

  const double m = (v - cy) / fy;
  const double den = std::tan(std::atan(m) + tilt_rad);
  if (den <= 1e-6) {return g;}   // 지평선 위 또는 그 선상

  const double d = h / den;
  g.x_rear = d + rear_offset;
  g.lat_scale = std::hypot(h, d) / fx;
  g.lane_px_row = lane_width_m / g.lat_scale;

  const double w = m * std::cos(tilt_rad) + std::sin(tilt_rad);
  const double long_scale = h / (fy * w * w);
  g.lane_px_col = lane_width_m / long_scale;

  g.valid = true;
  return g;
}

// ROI 행 구간의 사전계산 테이블.
struct RowTable
{
  int row_min = 0;
  int row_max = 0;                 // 포함
  std::vector<RowGeometry> rows;

  bool contains(int v) const {return v >= row_min && v <= row_max;}
  const RowGeometry & at(int v) const {return rows[static_cast<size_t>(v - row_min)];}
};

inline RowTable buildRowTable(
  int row_min, int row_max, double fx, double fy, double cy, double h,
  double tilt_rad, double rear_offset, double lane_width_m)
{
  RowTable t;
  t.row_min = row_min;
  t.row_max = row_max;
  t.rows.reserve(static_cast<size_t>(std::max(0, row_max - row_min + 1)));

  // 행 중심을 쓴다 (픽셀 v 는 [v, v+1) 을 덮는다).
  for (int v = row_min; v <= row_max; ++v) {
    t.rows.push_back(
      rowGeometry(v + 0.5, fx, fy, cy, h, tilt_rad, rear_offset, lane_width_m));
  }
  return t;
}

}  // namespace backup_lane_detection

#endif  // BACKUP_LANE_DETECTION__HOMOGRAPHY_HPP_
