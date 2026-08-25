// 조명 불변 흰선 마스크 + 런 검출.
//
// 절대 밝기 임계는 쓰지 않는다. 전역 조명이 변하면 흰선이 임계 위아래를
// 넘나든다. top-hat 은 커널보다 넓은 배경 밝기를 전부 빼므로 가산성 조명
// 변화에 불변이고, 뒤의 임계를 고정값으로 둘 수 있다.

#ifndef BACKUP_LANE_DETECTION__WHITE_MASK_HPP_
#define BACKUP_LANE_DETECTION__WHITE_MASK_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "backup_lane_detection/homography.hpp"

namespace backup_lane_detection
{

// ROI 상대 행 구간 + 그 구간의 top-hat 커널 폭.
struct Strip
{
  int row0 = 0;      // 포함, ROI 상대
  int row1 = 0;      // 미포함
  int kernel = 3;    // 홀수
  double lane_px = 0.0;
};

inline int toOdd(double w)
{
  int k = static_cast<int>(std::lround(w));
  if (k < 3) {k = 3;}
  if ((k & 1) == 0) {++k;}
  return k;
}

// 흰선 픽셀 폭이 ROI 안에서 5 배 넘게 변하므로 커널 하나로는 안 된다.
// 경계는 선폭이 등비로 늘도록 잡는다 -- 각 스트립의 폭 오차가 균등해진다.
inline std::vector<Strip> buildStrips(
  const RowTable & rt, int n_strips, double kernel_ratio)
{
  std::vector<Strip> out;
  if (rt.rows.empty() || n_strips < 1) {return out;}

  const int n_rows = static_cast<int>(rt.rows.size());

  // 유효 행 구간 (지평선 위는 버린다).
  int first = -1;
  int last = -1;
  for (int i = 0; i < n_rows; ++i) {
    if (rt.rows[static_cast<size_t>(i)].valid) {
      if (first < 0) {first = i;}
      last = i;
    }
  }
  if (first < 0) {return out;}

  const double w_lo = rt.rows[static_cast<size_t>(first)].lane_px_row;
  const double w_hi = rt.rows[static_cast<size_t>(last)].lane_px_row;

  std::vector<int> bound;
  bound.push_back(first);
  if (w_hi > w_lo * 1.0001) {
    for (int k = 1; k < n_strips; ++k) {
      const double target =
        w_lo * std::pow(w_hi / w_lo, static_cast<double>(k) / n_strips);
      int r = bound.back();
      while (r <= last && rt.rows[static_cast<size_t>(r)].lane_px_row < target) {++r;}
      if (r > bound.back() && r <= last) {bound.push_back(r);}
    }
  }
  bound.push_back(last + 1);

  for (size_t i = 0; i + 1 < bound.size(); ++i) {
    Strip s;
    s.row0 = bound[i];
    s.row1 = bound[i + 1];
    if (s.row1 <= s.row0) {continue;}
    // 스트립 중앙 행의 선폭을 대표값으로 쓴다.
    s.lane_px = rt.rows[static_cast<size_t>((s.row0 + s.row1 - 1) / 2)].lane_px_row;
    s.kernel = toOdd(s.lane_px * kernel_ratio);
    out.push_back(s);
  }
  return out;
}

struct MaskParams
{
  int tophat_threshold = 30;
  int saturation_max = 60;
  bool bidirectional = true;
  bool use_clahe = false;
  double clahe_clip = 2.0;
  int clahe_grid = 8;
  int morph_open = 2;
  int morph_close = 3;
};

// roi_bgr -> mask(0/255) + tophat(가중치용 응답).
// cvtColor(BGR2HLS) 를 부르지 않는다 -- max/min(B,G,R) 로 L 과 채도를 동시에 낸다.
inline void buildWhiteMask(
  const cv::Mat & roi_bgr, const std::vector<Strip> & strips,
  const MaskParams & p, cv::Mat & mask, cv::Mat & tophat)
{
  std::vector<cv::Mat> ch;
  cv::split(roi_bgr, ch);

  cv::Mat mx;
  cv::Mat mn;
  cv::max(ch[0], ch[1], mx);
  cv::max(mx, ch[2], mx);
  cv::min(ch[0], ch[1], mn);
  cv::min(mn, ch[2], mn);

  cv::Mat lum;
  cv::addWeighted(mx, 0.5, mn, 0.5, 0.0, lum);   // HLS 의 L

  cv::Mat chroma;
  cv::subtract(mx, mn, chroma);                  // HLS 의 채도 대용

  if (p.use_clahe) {
    // 곱셈성 조명 변화까지 잡을 때만. 비용이 붙는다.
    const int g = std::max(1, p.clahe_grid);
    cv::createCLAHE(p.clahe_clip, cv::Size(g, g))->apply(lum, lum);
  }

  tophat = cv::Mat::zeros(lum.size(), CV_8UC1);

  cv::Mat th_h;
  cv::Mat th_v;
  for (const Strip & s : strips) {
    const int k = s.kernel;
    // 수직 커널은 스트립 경계에서 border replication 인공물을 만든다.
    // 커널 반폭만큼 위아래를 더 읽고 잘라낸다.
    const int pad = p.bidirectional ? k / 2 : 0;
    const int p0 = std::max(0, s.row0 - pad);
    const int p1 = std::min(lum.rows, s.row1 + pad);
    if (p1 <= p0) {continue;}

    const cv::Mat sub = lum.rowRange(p0, p1);

    cv::morphologyEx(
      sub, th_h, cv::MORPH_TOPHAT,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, 1)));

    if (p.bidirectional) {
      // 급코너에서 차선이 영상에서 가로로 눕는다. 수평 커널만 쓰면
      // 그 구간을 통째로 놓친다.
      cv::morphologyEx(
        sub, th_v, cv::MORPH_TOPHAT,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, k)));
      cv::max(th_h, th_v, th_h);
    }

    th_h.rowRange(s.row0 - p0, s.row1 - p0)
    .copyTo(tophat.rowRange(s.row0, s.row1));
  }

  cv::threshold(
    tophat, mask, static_cast<double>(p.tophat_threshold), 255.0, cv::THRESH_BINARY);

  // 노란선/잔디/주황콘 배제.
  cv::Mat achromatic;
  cv::threshold(
    chroma, achromatic, static_cast<double>(p.saturation_max), 255.0,
    cv::THRESH_BINARY_INV);
  cv::bitwise_and(mask, achromatic, mask);

  if (p.morph_open > 1) {
    cv::morphologyEx(
      mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(p.morph_open, p.morph_open)));
  }
  if (p.morph_close > 1) {
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(p.morph_close, p.morph_close)));
  }
}

struct RunParams
{
  bool scan_rows = true;
  bool scan_cols = true;
  double tol_lo = 0.5;
  double tol_hi = 2.0;
  int min_run_pixels = 3;
  int row_step = 1;
  int col_step = 2;
};

// 영상좌표 서브픽셀 센트로이드.
struct RunPoint
{
  double u = 0.0;
  double v = 0.0;
};

// 슬라이딩 윈도우 argmax 는 1 px 양자화가 그대로 잡음이 된다.
// 런 센트로이드는 1/sqrt(12N) px 이고 윈도우 탐색도 없다.
// 45 deg 부근 차선은 행/열 양쪽에 잡히며 런 폭이 1.41 배가 되므로
// tol_hi 2.0 안에 들어온다. 중복은 호출자가 지면 격자로 병합한다.
inline void extractRuns(
  const cv::Mat & mask, const cv::Mat & weight, const RowTable & rt,
  int roi_row0, const RunParams & rp, std::vector<RunPoint> & out)
{
  const int rows = mask.rows;
  const int cols = mask.cols;

  if (rp.scan_rows) {
    const int step = std::max(1, rp.row_step);
    for (int r = 0; r < rows; r += step) {
      const int v = roi_row0 + r;
      if (!rt.contains(v)) {continue;}
      const RowGeometry & g = rt.at(v);
      if (!g.valid || g.lane_px_row <= 0.0) {continue;}

      const double lo =
        std::max(static_cast<double>(rp.min_run_pixels), g.lane_px_row * rp.tol_lo);
      const double hi = g.lane_px_row * rp.tol_hi;
      if (hi < lo) {continue;}

      const uchar * m = mask.ptr<uchar>(r);
      const uchar * w = weight.ptr<uchar>(r);

      int c = 0;
      while (c < cols) {
        if (m[c] == 0) {++c; continue;}
        const int s = c;
        while (c < cols && m[c] != 0) {++c;}
        const int len = c - s;
        if (len < lo || len > hi) {continue;}

        double sw = 0.0;
        double su = 0.0;
        for (int i = s; i < c; ++i) {
          // +1 은 top-hat 응답이 전부 0 인 퇴화 런에서 0 나눗셈을 막는다.
          const double ww = static_cast<double>(w[i]) + 1.0;
          sw += ww;
          su += ww * (static_cast<double>(i) + 0.5);
        }
        out.push_back({su / sw, static_cast<double>(v) + 0.5});
      }
    }
  }

  if (rp.scan_cols) {
    const int step = std::max(1, rp.col_step);
    for (int c = 0; c < cols; c += step) {
      int r = 0;
      while (r < rows) {
        if (mask.at<uchar>(r, c) == 0) {++r; continue;}
        const int s = r;
        while (r < rows && mask.at<uchar>(r, c) != 0) {++r;}
        const int len = r - s;

        const int vc = roi_row0 + (s + r - 1) / 2;
        if (!rt.contains(vc)) {continue;}
        const RowGeometry & g = rt.at(vc);
        if (!g.valid || g.lane_px_col <= 0.0) {continue;}

        const double lo =
          std::max(static_cast<double>(rp.min_run_pixels), g.lane_px_col * rp.tol_lo);
        const double hi = g.lane_px_col * rp.tol_hi;
        if (len < lo || len > hi) {continue;}

        double sw = 0.0;
        double sv = 0.0;
        for (int i = s; i < r; ++i) {
          const double ww = static_cast<double>(weight.at<uchar>(i, c)) + 1.0;
          sw += ww;
          sv += ww * (static_cast<double>(roi_row0 + i) + 0.5);
        }
        out.push_back({static_cast<double>(c) + 0.5, sv / sw});
      }
    }
  }
}

}  // namespace backup_lane_detection

#endif  // BACKUP_LANE_DETECTION__WHITE_MASK_HPP_
