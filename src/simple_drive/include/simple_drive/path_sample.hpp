// ====================================================================
// path_sample.hpp
//
// BackupPath(quintic Bezier 제어점 열) -> 전방 lookahead 지점.
// 헤더 온리, 상태 없음.
//
// 좌표계는 BackupPath 그대로다: base_link, 원점 후륜축,
// x 전방 [m], y 좌측 + [m].
// ====================================================================

#ifndef SIMPLE_DRIVE__PATH_SAMPLE_HPP_
#define SIMPLE_DRIVE__PATH_SAMPLE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <backup_msgs/msg/backup_path.hpp>

namespace simple_drive
{

struct P2
{
  double x = 0.0;
  double y = 0.0;
};

struct Lookahead
{
  bool valid = false;
  P2 point;              // lookahead 지점 [m]
  double s = 0.0;        // 실제로 쓴 호길이 [m]. 요청값보다 짧을 수 있다
  double lateral = 0.0;  // 그 지점의 횡오차 = point.y. 좌측 +
  double heading = 0.0;  // 그 지점까지의 방위 atan2(y, x) [rad]. 좌측 +
};

// 한 세그먼트를 De Casteljau 로 평가한다. ctrl 은 (degree+1) 개.
inline P2 evalSeg(const double * cx, const double * cy, int degree, double u)
{
  // degree 는 5 고정이지만 상수를 박지 않는다 -- 발행측이 바뀌어도
  // 조용히 어긋나는 것보다 낫다.
  std::vector<double> bx(cx, cx + degree + 1);
  std::vector<double> by(cy, cy + degree + 1);
  for (int k = degree; k > 0; --k) {
    for (int i = 0; i < k; ++i) {
      bx[i] += u * (bx[i + 1] - bx[i]);
      by[i] += u * (by[i + 1] - by[i]);
    }
  }
  return P2{bx[0], by[0]};
}

// 경로 전체를 등간격 u 로 훑어 누적 현길이(chord length) LUT 을 만든다.
// Bezier 의 정확한 호길이는 Gauss-Legendre 가 필요하지만, 여기서
// 필요한 정밀도는 "몇 cm 앞을 보는가" 수준이라 현길이면 충분하다
// (samples 64 에서 오차 < 1 mm).
inline void sampleChain(
  const backup_msgs::msg::BackupPath & path,
  int samples_per_seg,
  std::vector<P2> & pts,
  std::vector<double> & cum)
{
  pts.clear();
  cum.clear();

  const int degree = static_cast<int>(path.degree);
  if (degree < 1) {return;}

  const std::size_t nctrl = static_cast<std::size_t>(degree) + 1;
  if (path.ctrl_x.size() != path.ctrl_y.size()) {return;}
  if (path.ctrl_x.empty() || path.ctrl_x.size() % nctrl != 0) {return;}

  const std::size_t nseg = path.ctrl_x.size() / nctrl;
  const int n = std::max(2, samples_per_seg);

  double acc = 0.0;
  for (std::size_t seg = 0; seg < nseg; ++seg) {
    const double * cx = path.ctrl_x.data() + seg * nctrl;
    const double * cy = path.ctrl_y.data() + seg * nctrl;
    // 세그먼트 경계 중복 방지: 첫 세그먼트만 u=0 을 넣는다.
    const int i0 = (seg == 0) ? 0 : 1;
    for (int i = i0; i <= n; ++i) {
      const P2 p = evalSeg(cx, cy, degree, static_cast<double>(i) / n);
      if (!pts.empty()) {
        acc += std::hypot(p.x - pts.back().x, p.y - pts.back().y);
      }
      pts.push_back(p);
      cum.push_back(acc);
    }
  }
}

// 호길이 s_req 지점을 찾는다.
//
// s_req 는 두 번 잘린다:
//   1) valid_length - margin  -- 검출 근거가 실제로 닿는 거리 밖은
//      외삽이라 쫓으면 안 된다 (backup_steer_controller 와 같은 규칙)
//   2) 경로 전체 길이
inline Lookahead lookaheadAt(
  const backup_msgs::msg::BackupPath & path,
  double s_req,
  double valid_margin_m,
  int samples_per_seg = 32)
{
  Lookahead out;

  std::vector<P2> pts;
  std::vector<double> cum;
  sampleChain(path, samples_per_seg, pts, cum);
  if (pts.size() < 2) {return out;}

  double s = s_req;
  if (path.valid_length > 0.0) {
    s = std::min(s, path.valid_length - valid_margin_m);
  }
  s = std::min(s, cum.back());
  if (s <= 1e-3) {return out;}

  // 선형 보간으로 s 지점을 뽑는다.
  std::size_t i = 1;
  while (i < cum.size() && cum[i] < s) {++i;}
  if (i >= cum.size()) {i = cum.size() - 1;}

  const double seg_len = cum[i] - cum[i - 1];
  const double t = (seg_len > 1e-9) ? (s - cum[i - 1]) / seg_len : 0.0;

  out.valid = true;
  out.point.x = pts[i - 1].x + t * (pts[i].x - pts[i - 1].x);
  out.point.y = pts[i - 1].y + t * (pts[i].y - pts[i - 1].y);
  out.s = s;
  out.lateral = out.point.y;
  out.heading = std::atan2(out.point.y, std::max(out.point.x, 1e-6));
  return out;
}

}  // namespace simple_drive

#endif  // SIMPLE_DRIVE__PATH_SAMPLE_HPP_
