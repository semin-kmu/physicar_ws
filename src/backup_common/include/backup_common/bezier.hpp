// quintic Bezier 경로. 헤더 온리. 단위 m.
//
// 원호 스플라인을 쓰지 않는 이유: 이음매마다 곡률이 계단으로 뛰어
// 조향각 계단이 생기고, 회피 S 자에서 특히 나쁘다.
// quintic 은 양 끝 제어점 3 개를 공선으로 두면 끝 곡률이 정확히 0 이라
// 직선 구간과 G2 로 이어진다.
//
// 호길이는 Gauss-Legendre 10 점. kau_lane_detection/bezier.hpp 와 같은 방식이나
// 단위가 cm 가 아니라 m 다.

#ifndef BACKUP_COMMON__BEZIER_HPP_
#define BACKUP_COMMON__BEZIER_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace backup_common
{

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

inline Point2 operator+(const Point2 & a, const Point2 & b) {return {a.x + b.x, a.y + b.y};}
inline Point2 operator-(const Point2 & a, const Point2 & b) {return {a.x - b.x, a.y - b.y};}
inline Point2 operator*(const Point2 & a, double s) {return {a.x * s, a.y * s};}

inline constexpr int kDegree = 5;
inline constexpr int kCtrlPerSeg = kDegree + 1;

// Gauss-Legendre 10 점 (구간 [-1,1])
inline const std::array<double, 5> kGLNode{
  0.1488743389816312, 0.4333953941292472, 0.6794095682990244,
  0.8650633666889845, 0.9739065285171717};
inline const std::array<double, 5> kGLWeight{
  0.2955242247147529, 0.2692667193099963, 0.2190863625159820,
  0.1494513491505806, 0.0666713443086881};

// 제어점 6 개짜리 segment 하나.
struct Segment
{
  std::array<Point2, kCtrlPerSeg> p{};

  Point2 eval(double t) const
  {
    const double u = 1.0 - t;
    const double b0 = u * u * u * u * u;
    const double b1 = 5.0 * t * u * u * u * u;
    const double b2 = 10.0 * t * t * u * u * u;
    const double b3 = 10.0 * t * t * t * u * u;
    const double b4 = 5.0 * t * t * t * t * u;
    const double b5 = t * t * t * t * t;
    return {
      p[0].x * b0 + p[1].x * b1 + p[2].x * b2 + p[3].x * b3 + p[4].x * b4 + p[5].x * b5,
      p[0].y * b0 + p[1].y * b1 + p[2].y * b2 + p[3].y * b3 + p[4].y * b4 + p[5].y * b5};
  }

  // P'(t) = 5 * sum (P[i+1]-P[i]) * B_i^4(t)
  Point2 deriv(double t) const
  {
    const double u = 1.0 - t;
    const double c0 = u * u * u * u;
    const double c1 = 4.0 * t * u * u * u;
    const double c2 = 6.0 * t * t * u * u;
    const double c3 = 4.0 * t * t * t * u;
    const double c4 = t * t * t * t;
    Point2 d{0.0, 0.0};
    const double w[5] = {c0, c1, c2, c3, c4};
    for (int i = 0; i < 5; ++i) {
      d.x += (p[i + 1].x - p[i].x) * w[i];
      d.y += (p[i + 1].y - p[i].y) * w[i];
    }
    return {d.x * 5.0, d.y * 5.0};
  }

  // P''(t) = 20 * sum (P[i+2]-2P[i+1]+P[i]) * B_i^3(t)
  Point2 deriv2(double t) const
  {
    const double u = 1.0 - t;
    const double w[4] = {u * u * u, 3.0 * t * u * u, 3.0 * t * t * u, t * t * t};
    Point2 d{0.0, 0.0};
    for (int i = 0; i < 4; ++i) {
      d.x += (p[i + 2].x - 2.0 * p[i + 1].x + p[i].x) * w[i];
      d.y += (p[i + 2].y - 2.0 * p[i + 1].y + p[i].y) * w[i];
    }
    return {d.x * 20.0, d.y * 20.0};
  }

  // 부호 있는 곡률 [1/m]. 좌회전 +.
  double curvature(double t) const
  {
    const Point2 d1 = deriv(t);
    const Point2 d2 = deriv2(t);
    const double sp = d1.x * d1.x + d1.y * d1.y;
    if (sp < 1e-12) {return 0.0;}
    return (d1.x * d2.y - d1.y * d2.x) / std::pow(sp, 1.5);
  }

  double length() const
  {
    double s = 0.0;
    for (int i = 0; i < 5; ++i) {
      for (int sign = -1; sign <= 1; sign += 2) {
        const double t = 0.5 + 0.5 * sign * kGLNode[i];
        const Point2 d = deriv(t);
        s += kGLWeight[i] * std::hypot(d.x, d.y);
      }
    }
    return s * 0.5;
  }

  // 표본 n 개에서 |kappa| 최대. n 은 홀수/짝수 무관.
  double maxAbsCurvature(int n = 21) const
  {
    double m = 0.0;
    for (int i = 0; i < n; ++i) {
      m = std::max(m, std::fabs(curvature(static_cast<double>(i) / (n - 1))));
    }
    return m;
  }
};

// segment 열 + 호길이 LUT.
// LUT 는 경로 수신 시 1 회 만들고 50 Hz 제어에서는 보간만 한다
// (Newton 반복을 50 Hz 로 돌리지 않는다).
class Path
{
public:
  void clear()
  {
    segs_.clear();
    lut_s_.clear();
    lut_t_.clear();
    lut_seg_.clear();
    total_ = 0.0;
  }

  void addSegment(const Segment & s) {segs_.push_back(s);}

  const std::vector<Segment> & segments() const {return segs_;}
  bool empty() const {return segs_.empty();}
  double totalLength() const {return total_;}

  // 전 segment 를 균일 t 로 표본해 (s, seg, t) LUT 를 만든다.
  void buildLut(int samples_per_seg = 64)
  {
    lut_s_.clear();
    lut_t_.clear();
    lut_seg_.clear();
    total_ = 0.0;
    if (segs_.empty()) {return;}

    Point2 prev = segs_[0].eval(0.0);
    lut_s_.push_back(0.0);
    lut_t_.push_back(0.0);
    lut_seg_.push_back(0);

    for (size_t k = 0; k < segs_.size(); ++k) {
      for (int i = 1; i <= samples_per_seg; ++i) {
        const double t = static_cast<double>(i) / samples_per_seg;
        const Point2 cur = segs_[k].eval(t);
        total_ += std::hypot(cur.x - prev.x, cur.y - prev.y);
        lut_s_.push_back(total_);
        lut_t_.push_back(t);
        lut_seg_.push_back(k);
        prev = cur;
      }
    }
  }

  // 호길이 s 지점. LUT 선형보간. s 는 [0, totalLength] 로 클램프된다.
  Point2 pointAtArcLength(double s) const
  {
    size_t i = lutIndex(s);
    if (i + 1 >= lut_s_.size()) {
      return segs_.back().eval(1.0);
    }
    const double s0 = lut_s_[i], s1 = lut_s_[i + 1];
    const double w = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;
    const Point2 a = segs_[lut_seg_[i]].eval(lut_t_[i]);
    const Point2 b = segs_[lut_seg_[i + 1]].eval(lut_t_[i + 1]);
    return {a.x + (b.x - a.x) * w, a.y + (b.y - a.y) * w};
  }

  double curvatureAtArcLength(double s) const
  {
    const size_t i = lutIndex(s);
    if (i >= lut_seg_.size()) {return 0.0;}
    return segs_[lut_seg_[i]].curvature(lut_t_[i]);
  }

  // [0, s_end] 구간의 |kappa| 최대.
  double maxAbsCurvatureUpTo(double s_end) const
  {
    double m = 0.0;
    for (size_t i = 0; i < lut_s_.size(); ++i) {
      if (lut_s_[i] > s_end) {break;}
      m = std::max(m, std::fabs(segs_[lut_seg_[i]].curvature(lut_t_[i])));
    }
    return m;
  }

  // 원점(자차)에서 경로까지의 부호 있는 횡오차. 좌측 +.
  double crossTrackAtOrigin() const
  {
    if (lut_s_.empty()) {return 0.0;}
    double best = 1e9;
    size_t bi = 0;
    for (size_t i = 0; i < lut_s_.size(); ++i) {
      const Point2 q = segs_[lut_seg_[i]].eval(lut_t_[i]);
      const double d = q.x * q.x + q.y * q.y;
      if (d < best) {best = d; bi = i;}
    }
    const Segment & sg = segs_[lut_seg_[bi]];
    const Point2 q = sg.eval(lut_t_[bi]);
    const Point2 d1 = sg.deriv(lut_t_[bi]);
    const double n = std::hypot(d1.x, d1.y);
    if (n < 1e-9) {return 0.0;}
    // 자차(원점)가 경로 왼쪽이면 +
    return -((0.0 - q.x) * (-d1.y) + (0.0 - q.y) * (d1.x)) / n;
  }

  // 모든 제어점에 함수 적용 (데드레커닝용).
  template<typename F>
  void applyToControlPoints(F f)
  {
    for (auto & s : segs_) {
      for (auto & p : s.p) {f(p.x, p.y);}
    }
  }

private:
  size_t lutIndex(double s) const
  {
    if (lut_s_.empty()) {return 0;}
    const double sc = std::max(0.0, std::min(s, total_));
    auto it = std::lower_bound(lut_s_.begin(), lut_s_.end(), sc);
    size_t i = static_cast<size_t>(it - lut_s_.begin());
    if (i > 0) {--i;}
    return i;
  }

  std::vector<Segment> segs_;
  std::vector<double> lut_s_;
  std::vector<double> lut_t_;
  std::vector<size_t> lut_seg_;
  double total_ = 0.0;
};

// Pure Pursuit. 기준점은 후륜축(원점), 목표는 body frame 좌표.
// delta = atan( 2*L*sin(alpha) / Ld ),  alpha = atan2(ty, tx)
inline double purePursuitSteer(double tx, double ty, double wheelbase, double ld)
{
  const double l = std::max(ld, 1e-6);
  const double alpha = std::atan2(ty, tx);
  return std::atan2(2.0 * wheelbase * std::sin(alpha), l);
}

}  // namespace backup_common

#endif  // BACKUP_COMMON__BEZIER_HPP_
