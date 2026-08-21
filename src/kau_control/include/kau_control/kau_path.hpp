// ====================================================================
// kau_path.hpp
//
// kau_msgs/KauPath  <->  control::Curve 변환.
//
// 시뮬 레퍼런스는 Curve.from_knots() 로 경로를 직접 만들었지만, ROS 에서는
// 경로가 message 로 들어온다. 그 경계가 이 파일이다.
//
// KauPath 는 cm 단위, 제어점 flat 배열 (kau_msgs/README.md 규약):
//     segment k 의 j 번째 제어점 = ctrl_x[k*(degree+1) + j]
//     len(ctrl_x) == len(ctrl_y) == len(seg_length) * (degree+1)
//     len(seg_kappa_max) == len(seg_length)
//     num_segments 필드는 없다. len(seg_length) 에서 유도.
// ====================================================================

#ifndef KAU_CONTROL__KAU_PATH_HPP_
#define KAU_CONTROL__KAU_PATH_HPP_

#include <string>
#include <vector>

#include "kau_msgs/msg/kau_path.hpp"

#include "kau_control/curve.hpp"


namespace kau
{
namespace control
{

// 무결성 검사 (kau_msgs/README.md §9.7~9.8).
// 통과하면 true, 아니면 reason 에 사유를 담고 false.
inline bool validateKauPath(
    const kau_msgs::msg::KauPath & msg,
    std::string & reason)
{
    if (msg.degree != bezier::DEGREE)
    {
        reason = "degree=" + std::to_string(static_cast<int>(msg.degree)) +
                 " (운용값 " + std::to_string(bezier::DEGREE) +
                 " 만 수신한다. 발행측이 degree elevation 후 발행할 것)";

        return false;
    }

    const std::size_t nseg = msg.seg_length.size();

    if (nseg == 0)
    {
        reason = "seg_length 가 비어 있다 (num_segments = 0)";

        return false;
    }

    const std::size_t expect = nseg * static_cast<std::size_t>(NCTRL);

    if (msg.ctrl_x.size() != expect || msg.ctrl_y.size() != expect)
    {
        reason = "제어점 개수 불일치: ctrl_x=" +
                 std::to_string(msg.ctrl_x.size()) + " ctrl_y=" +
                 std::to_string(msg.ctrl_y.size()) + " 기대=" +
                 std::to_string(expect);

        return false;
    }

    if (msg.seg_kappa_max.size() != nseg)
    {
        reason = "seg_kappa_max 길이=" +
                 std::to_string(msg.seg_kappa_max.size()) +
                 " seg_length 길이=" + std::to_string(nseg);

        return false;
    }

    return true;
}


// KauPath -> Curve. 호출 전에 validateKauPath 로 검사할 것.
//
// 단위 변환은 하지 않는다. KauPath 도 Curve 도 cm 이므로 그대로 옮긴다.
inline Curve curveFromKauPath(const kau_msgs::msg::KauPath & msg)
{
    const std::size_t nseg = msg.seg_length.size();

    std::vector<Ctrl> segs;

    segs.reserve(nseg);


    for (std::size_t k = 0; k < nseg; ++k)
    {
        Ctrl c;

        c.reserve(static_cast<std::size_t>(NCTRL));

        for (int j = 0; j < NCTRL; ++j)
        {
            const std::size_t idx =
                k * static_cast<std::size_t>(NCTRL) +
                static_cast<std::size_t>(j);

            c.push_back(Point2{msg.ctrl_x[idx], msg.ctrl_y[idx]});
        }

        segs.push_back(std::move(c));
    }


    return Curve(std::move(segs), msg.is_closed);
}

}  // namespace control
}  // namespace kau

#endif  // KAU_CONTROL__KAU_PATH_HPP_
