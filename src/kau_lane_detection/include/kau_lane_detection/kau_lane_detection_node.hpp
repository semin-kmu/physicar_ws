    // ====================================================================
    // kau_lane_detection_node.hpp
    //
    // 클래스 선언만. 구현은 kau_lane_detection.cpp, 진입점은
    // kau_lane_detection_node.cpp 에 있다.
    // ====================================================================

    #ifndef KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_
    #define KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_

    #include <rclcpp/rclcpp.hpp>

    #include <sensor_msgs/msg/image.hpp>
    #include <sensor_msgs/msg/camera_info.hpp>
    #include <sensor_msgs/msg/joint_state.hpp>
    #include <std_msgs/msg/string.hpp>

    // Pan 명령 (/camera/pan, 절대각 rad)
    #include <std_msgs/msg/float64.hpp>

    // 팀 공용 함수형 경로 (docs/경로_형식.md 9장)
    #include <kau_msgs/msg/kau_path.hpp>

    // kau_global_path 가 latched 로 내는 전역 경로. 곡률 룩어헤드에 쓴다.
    // KauPath 그대로이므로 별도 타입은 없다.

    // kau_object_detection 이 map 프레임으로 내는 장애물 원(circle) 목록.
    // pan 트리거의 "몇 m 앞에 장애물" 판정에 쓴다.
    #include <kau_msgs/msg/obstacle_circle_array.hpp>

    // RViz 는 custom message 를 못 그린다. 시각화 전용 이산화 발행 (9.9)
    #include <nav_msgs/msg/path.hpp>

    // map <- base_link. 경로를 map 프레임으로 내보내고, /path/global 및
    // /perception/obstacles(둘 다 map 프레임)와 비교하려면 자차의 map 위치가
    // 필요하다 (측위 의존 — CLAUDE.md §1 개정 근거 참고).
    #include <tf2_ros/buffer.hpp>
    #include <tf2_ros/transform_listener.hpp>

    #include <opencv2/opencv.hpp>

    #include "kau_lane_detection/bezier.hpp"

    // 이 맵의 차선 정보 (전역 선언). 차로 수, 도로 단면의 선 목록,
    // 각 선의 중앙선 대비 횡거리와 색. 관측 하나를 단면의 한 자리에
    // 앉히려면 이것이 먼저 있어야 한다.
    #include "kau_lane_detection/road_map.hpp"

    #include <cstdint>
    #include <memory>
    #include <deque>
    #include <string>
    #include <vector>


    class KauLaneDetectionNode : public rclcpp::Node
    {
    public:

        // ================================================================
        // Lane Detection Result
        // ================================================================

        struct LaneDetectionResult
        {
            // 방향성 창 추적이 잡은 차선 중심점.
            // 차량에 가까운 쪽부터 전방으로 순서대로. [BEV px]
            //
            // 예전에는 창 y 격자에 묶인 x 배열이었다. 그 표현은
            // x = f(y) 를 전제하므로 90도 코너처럼 차선이 가로로
            // 누우면 표현 자체가 불가능하다. 순서 있는 점열은
            // 방향에 무관하다.
            std::vector<cv::Point2d> track_px;

            // 실제로 픽셀을 잡은 스텝 수 (= track_px.size()).
            int found_count = 0;

            bool detected = false;

            // track_px 를 중심선 합성에 쓸 수 있는 상태
            bool valid = false;
        };


        // 생성자. 파라미터 선언 + publisher/subscriber 생성.
        KauLaneDetectionNode();


    private:

        // ================================================================
        // Sliding Window Parameters
        //
        // 첨부한 코드의 구조를 기준으로 설정
        // nwindows = 9
        // margin   = 20
        // minpix   = 5
        // ================================================================

        static constexpr int NUM_WINDOWS = 9;

        static constexpr int MIN_VALID_WINDOWS = 3;


        // 가중 최소제곱 다항식 적합 v = f(t). Vandermonde + DECOMP_QR.
        // weights 가 비어 있으면 균등 가중.
        static bool polyFitW(
            const std::vector<double> & ts,
            const std::vector<double> & vs,
            const std::vector<double> & weights,
            int order,
            std::vector<double> & coeffs);


        // 폴리라인 위로 투영. 호길이 s 와 거리 d 를 돌려준다.
        // 양 끝 구간에서는 바깥으로 외삽을 허용한다 (s < 0 가능).
        static void projectOnPolyline(
            const std::vector<cv::Point2d> & poly,
            const std::vector<double> & cum,
            const cv::Point2d & p,
            double & s_out,
            double & d_out);


        // pts 각 점의 ref 기준 부호 있는 횡거리 [px]. 차량 오른쪽이 +.
        static std::vector<double> lateralOffsets(
            const std::vector<cv::Point2d> & ref,
            const std::vector<cv::Point2d> & pts);


        // 위 값의 중앙값. 차선이 통째로 어느 쪽에 있는지 판정용.
        static double medianLateralOffset(
            const std::vector<cv::Point2d> & ref,
            const std::vector<cv::Point2d> & pts);


        // ref 기준 회랑 [lo, hi] * lane_width 를 벗어나는 순간 잘라낸다.
        // 추적이 체커보드 연석이나 다른 구간 차선으로 갈아탄 지점에서
        // 끊는다. 통째로 버리지 않고 정상이던 앞부분은 남긴다.
        static std::vector<cv::Point2d> truncateAtCorridor(
            const std::vector<cv::Point2d> & track,
            const std::vector<cv::Point2d> & ref,
            double want_sign,
            double lo_px,
            double hi_px);


        // 순서 있는 점열을 국소 법선(차량 오른쪽) 방향으로 평행이동.
        // 90도 코너에서는 x 방향 offset 이 틀리므로 반드시 법선이어야 한다.
        static std::vector<cv::Point2d> offsetTrack(
            const std::vector<cv::Point2d> & pts,
            double offset_px);


        // 파라미터(int 배열) -> cv::Scalar.
        static cv::Scalar toScalar(
            const std::vector<int64_t> & v);


        // Horner 평가.
        static double polyEval(
            const std::vector<double> & coeffs,
            double y);


        // src 사다리꼴 -> dst 사각형 변환행렬 생성.
        void updateBevGeometry();


        // 파라미터 변화 감지. 바뀐 프레임에만 행렬 재계산.
        void refreshBevParameters();


        // ------------------------------------------------------------
        // 콜백 예외 가드
        //
        // 콜백에서 새어 나간 예외는 rclcpp::spin() 을 그대로 뚫고
        // 나가 **노드 프로세스를 죽인다**. 그 순간 /lane/center 가
        // 끊기고 차선 추종이 통째로 멈춘다.
        //
        // imageCallback 안쪽에는 cv_bridge / cv::Exception 처리가
        // 이미 있었지만 나머지 네 콜백(cameraInfo / jointState /
        // globalPath / obstacles)에는 아무 방어가 없었다. 그쪽도
        // 예외를 던질 수 있다 — 예를 들어 cameraInfoCallback 은
        // 손상된 CameraInfo 로 cv::Mat 를 만들다 cv::Exception 을,
        // jointStateCallback 은 rclcpp::Time 연산에서
        // std::runtime_error 를 낼 수 있다.
        //
        // 그래서 **구독 지점에서** 전부 이 가드로 감싼다. 함수 본문을
        // 건드리지 않으므로 기존 코드/주석이 그대로 남고, "모든 콜백이
        // 감싸져 있다"는 것을 생성자 한 곳만 보면 확인할 수 있다.
        //
        // 한 프레임을 버리는 것이지 상태를 복구하지는 않는다. 같은
        // 예외가 계속 나면 로그가 THROTTLE 로 남으므로, 조용히 죽는
        // 대신 원인을 남기고 계속 도는 쪽을 택한 것이다.
        // ------------------------------------------------------------
        template <typename Fn>
        void guardCallback(
            const char * where,
            Fn && fn)
        {
            try
            {
                fn();
            }
            catch (const cv::Exception & e)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "%s: OpenCV 예외로 이번 메시지를 버립니다: %s",
                    where,
                    e.what()
                );
            }
            catch (const std::exception & e)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "%s: 예외로 이번 메시지를 버립니다: %s",
                    where,
                    e.what()
                );
            }
            catch (...)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "%s: 알 수 없는 예외로 이번 메시지를 버립니다.",
                    where
                );
            }
        }


        // CameraInfo 수신 -> K, D.
        void cameraInfoCallback(
            const sensor_msgs::msg::CameraInfo::SharedPtr msg);


        // BEV 하단 히스토그램 peak. 슬라이딩 윈도우 시작점.
        // band_used 에 실제로 쓰인 밴드 비율을 돌려준다(폴백 여부 확인용).
        int findHistogramPeak(
            const cv::Mat & mask,
            int x_start,
            int x_end,
            double * band_used = nullptr);


        // 방향성 슬라이딩 윈도우 추적.
        //
        // 창이 항상 위(-y)로만 올라가는 대신, 지금까지의 진행 방향
        // theta 로 회전하며 그 방향으로 전진한다. 차선이 가로로
        // 누워도 창이 차선을 따라 돌아간다.
        LaneDetectionResult detectLaneSlidingWindow(
            const cv::Mat & mask,
            int base_x,
            cv::Mat & debug_image,
            const cv::Scalar & window_color);


        // ================================================================
        // 공통 경로 형식 — quintic Bezier 제어점
        //
        // docs/경로_형식.md 규약
        //
        //   1. 경로는 제어점으로만 생성/계산/출력. 이산 좌표 금지.
        //   2. 발행 차수는 항상 5.
        //   3. 내부 적합 차수는 달라도 되며, 낮은 차수로 적합했다면
        //      degree elevation 으로 5차로 승격해 발행 (무손실).
        //
        // 여기서 내부 적합은 매개변수 s (누적 호길이) 에 대한
        // x(s), y(s) 이고 차수는 최대 3차다. 이유:
        //
        //   x = f(y) 형태는 차선이 가로로 누우면 dx/dy = 무한대라
        //   표현 자체가 불가능하다. 90도 코너가 통째로 끊기던
        //   원인이었다. s 로 매개화하면 방향 전제가 사라진다.
        //
        //   90도 코너는 s 에 대한 사분원이라 2차로는 모자라고
        //   3차면 충분하다. 3차 매개곡선은 t 에 대해서도 정확히
        //   3차이므로, 멱기저 -> Bernstein -> degree elevation 이
        //   전부 정확한 기저 변환이다. 근사가 한 번도 개입하지 않는다.
        //
        // 이 경로는 차로를 고르지 않는다. 아래 sources (buildCenterlinePath
        // 참고) 는 왼쪽 흰선/노란 중앙선/오른쪽 흰선 세 후보를 전부
        // "중앙선의 위치 추정치"로 환산해 가중합하므로, 애초에 자차가
        // 어느 차로에 있는지 묻지 않고 중앙선만 추종한다.
        // ================================================================


        struct LanePath
        {
            // 제어점 6개 [cm]. 이것이 발행 대상의 전부다.
            kau::bezier::Ctrl ctrl;

            bool valid = false;

            // 아래는 전부 진단용 (발행하지 않음)
            double length_cm = 0.0;

            double kappa_max = 0.0;

            // 실관측 구간 [cm]. 근거 점이 끊기지 않고 이어지는
            // 시점부터의 호길이. 그 뒤는 외삽이다. KauPath 로 발행한다.
            double valid_length_cm = 0.0;

            // 0.0 ~ 1.0. 근거량 x 관측 비율.
            double confidence = 0.0;

            double cte_cm = 0.0;

            double heading_err = 0.0;

            int source_windows = 0;

            // 제어점 생성까지는 끝난 상태. 게이트 통과 여부와 무관하다.
            // 기각된 경로도 debug 화면에는 그려서 무엇이 막혔는지 보인다.
            bool built = false;

            // 게이트 기각 사유. 0 = 통과
            //   1 근거부족  2 조향한계  3 시야이탈
            int reject = 0;
        };


        // ================================================================
        // 차선 정보 — 도로 모형
        //
        // 선언 자체는 road_map.hpp 에 있다 (namespace kau_road).
        // 도로 단면의 선 목록, 각 선의 중앙선 대비 횡거리, 색.
        //
        // 이 코드는 차로를 지정하지 않는다. 자차가 중앙선 왼쪽
        // 차선에 있든 오른쪽 차선에 있든 상관하지 않고, 중앙선
        // (황색 점선)을 그대로 추종 목표로 삼는다. 그래서 road_map.hpp
        // 에는 "차로 번호"나 "지정 차선" 개념이 없다.
        //
        // 그래도 단면 선언 자체는 필요하다. 선이 하나만 보일 때
        // 그것이 어느 선인지 고를 근거가 있어야 하기 때문이다 —
        // 흰선 하나만 잡혔을 때 그게 왼쪽 도로 경계인지 오른쪽
        // 도로 경계인지 알아야, 거기서 중앙선이 어느 쪽으로
        // 31.25 cm 떨어져 있는지 역산할 수 있다 (identifyLine).
        //
        // 맵 mesh 실측 (custom_71e69ee938032295503bfed557fde18c.dae,
        // 삼각형을 x = 6.0 m 평면으로 잘라 단면을 낸 값. 단위 m):
        //
        //   road   y 0.8805 .. 1.5805    아스팔트 폭 0.700
        //   ol     y 0.8805 .. 0.9555    흰 실선   폭 0.075
        //   cl     y 1.2180 .. 1.2430    황색 점선 폭 0.025
        //   il     y 1.5055 .. 1.5805    흰 실선   폭 0.075
        //
        //   중앙선 중심 <-> 흰선 중심 = 31.25 cm   (= 차로 폭)
        //   흰선 중심   <-> 흰선 중심 = 62.50 cm   (= 도로 폭)
        //
        // 도로는 닫힌 고리다. 같은 x = 6.0 평면이 도로를 두 번 지나고
        // 두 통과의 선 순서가 서로 뒤집혀 있다 (아래쪽 ol-cl-il,
        // 위쪽 il-cl-ol). il 이 양쪽에서 안쪽을 향하므로 ol 이 바깥
        // 경계, il 이 안쪽 경계인 무한 루프다.
        //
        // 흰선은 차선 경계가 아니라 도로의 양 경계이고, 도로 안쪽의
        // 유일한 경계가 중앙선이다. 두 흰선 중 어느 쪽이 트랙 바깥
        // 경계인지는 루프를 도는 방향에 달렸고, 검출에는 쓰이지
        // 않는다 (차량 기준 좌/우만 쓴다).
        //
        // 같은 값을 영상에서도 확인했다. 직선 구간 한 프레임을
        // undistort 해 지면으로 역투영하면 (h = 14.65 cm)
        //
        //   도로 바깥 가장자리 간격 70.0 cm  ->  h 14.552
        //   흰선 안쪽 가장자리 간격 55.0 cm  ->  h 14.670
        //   중앙선 <-> 왼쪽 흰선 안쪽 27.5 cm -> h 14.708
        //   중앙선 <-> 오른쪽 흰선 안쪽 27.5 cm -> h 14.633
        //
        // 같은 프레임의 BEV 에서 흰선 사이 간격이 195.0 px 이고
        // sx = 0.3220 cm/px 이므로 62.79 cm. 실측 62.50 과 0.5% 차이.
        // ================================================================


        // 중앙선 대비 자차 위치의 최근 실측 (진단 + identifyLine prior 용).
        //
        // 예전에는 이 값으로 "차로 번호"(0/1)와 "지정 차선 이탈 여부"를
        // 판정해 EgoLane 으로 발행했다. 이제 차로 개념이 없으므로 그
        // 판정은 없앴다. 남은 건 순수하게 "중앙선에서 자차가 얼마나
        // 떨어져 있는가" 라는 진단값과, identifyLine 이 흰선 하나를
        // 왼쪽/오른쪽 경계 중 어디에 앉힐지 고를 때 쓰는 기준점뿐이다.
        struct CenterFix
        {
            bool valid = false;

            // 중앙선 -> 자차 부호 있는 횡거리 [cm]. 차량 오른쪽이 +.
            double offset_cm = 0.0;

            // 판정 근거
            //   0 없음
            //   1 노란 중앙선 실측
            //   2 흰선 두 개 사이를 반으로 갈라 역산
            //   3 흰선 하나 + 맵 단면으로 역산
            //   4 직전 프레임 유지 (이번 프레임 근거 없음)
            int source = 0;
        };


        // 기준선 대비 부호 있는 횡거리 하나. 차량 오른쪽이 +.
        //
        // lateralOffsets 는 투영 t 를 [0,1] 로 자르지만, 자차 위치는
        // 추적 시작점보다 뒤(BEV 아래쪽)에 있어서 그대로 쓰면 첫 점
        // 까지의 직선거리가 나와 값이 부풀려진다. 양 끝 구간에서는
        // 외삽을 허용한다.
        static double lateralOffsetAt(
            const std::vector<cv::Point2d> & ref,
            const cv::Point2d & p);


        // 중앙선 대비 자차 횡거리를 갱신한다.
        //
        // 반드시 흰선 검증(16-a) 뒤, 소실 복원(16-c) 앞에서 부른다.
        // 복원된 차선은 관측이 아니라 "차로 폭만큼 옆에 있을 것"
        // 이라는 가정이므로, 그것으로 이 값을 재면 가정을 관측
        // 처럼 되돌려 읽는 순환이 된다.
        //
        // center_fix_ 이력(유지 프레임 수)을 갱신하고, 확실한 근거
        // (노란선 실측 또는 흰선 두 개)일 때만 centerOffsetPriorCm()
        // 의 prior 도 함께 갱신한다.
        CenterFix resolveCenterOffset(
            const LaneDetectionResult & left,
            const LaneDetectionResult & yellow,
            const LaneDetectionResult & right,
            int bev_height);


        // BEV 상의 자차 종축 폴리라인 (아래 -> 위, 진행방향).
        // 관측선이 자차 기준 어느 쪽에 얼마나 있는지 재는 기준선.
        std::vector<cv::Point2d> egoAxis(int bev_height) const;


        // 관측선 하나를 도로 단면(road_map.hpp 의 LINES)의 한 자리에
        // 앉힌다. 반환값은 LINES 의 index, 못 고르면 -1.
        //
        // 선이 하나만 보일 때 그것이 왼쪽 가장자리인지 오른쪽
        // 가장자리인지는 관측만으로는 알 수 없다. 히스토그램이
        // 화면 반쪽 중 어디서 찾았는지는 근거가 못 된다. 코너에서
        // 차선이 가로로 누우면 화면 왼쪽에 오른쪽 선이 온다.
        //
        // 맵 단면이 있으면 답이 나온다. 자차가 중앙선에서 몇 cm
        // 떨어져 있는지(ego_offset_cm) 를 알면 각 선이 자차 종축
        // 에서 몇 cm 떨어져 있어야 하는지가 정해지고, 실측 횡거리를
        // 거기에 맞추면 된다. 이 맵에서 두 흰선의 기대 위치는
        // 62.5cm 떨어져 있으므로 판정 여유가 넉넉하다.
        //
        // 기대 위치의 기준점 ego_offset_cm 은 centerOffsetPriorCm() 이
        // 준다. 노란선/흰선 두 개로 잰 값이 싱싱하면 그것을 쓰고,
        // 없으면 중앙선 위에 있다고 둔다 (차로 개념이 없으므로 이
        // 값이 유일하게 근거 없는 기본값이다). 흰선 하나로 얻은
        // 값은 들어가지 않는다 (자기 출력 되먹임 방지).
        int identifyLine(
            const std::vector<cv::Point2d> & track,
            kau_road::LineType type,
            double ego_offset_cm,
            double sx,
            int bev_height) const;


        // identifyLine 에 넣을 "자차가 중앙선에서 몇 cm 에 있는가".
        //
        // 노란선이나 흰선 두 개로 잰 값이 아직 싱싱하면 그것을 쓰고,
        // 없으면 중앙선 위에 있다고 둔다(0.0). 차로 개념이 없으므로
        // "지정 차선 중심" 같은 기본값은 없다 — 중앙선 자체가 목표이니
        // 근거가 없을 때의 최선의 가정도 중앙선이다.
        //
        // 흰선 하나로 얻은 값(source 3)은 절대 여기 들어가지 않는다.
        // 그 값은 이 prior 로 자리를 골라서 나온 것이라, 되먹이면
        // 오차가 스스로를 키운다.
        double centerOffsetPriorCm() const;


        // ================================================================
        // Pan 기반 Lane Lost Recovery
        //
        // 한쪽 흰선이 연속으로 안 잡히면 그 방향으로 카메라를 돌려
        // 찾고, 찾으면(또는 상한까지 못 찾으면) 0 으로 복귀한다.
        //
        // ★ pan != 0 인 프레임도 이제 경로를 짓는다 (회전 보정).
        //
        //   updateBevGeometry() 의 src 사다리꼴은 "카메라가 정면을
        //   본다" 를 가정한 고정값이라 픽셀 <-> BEV 대응 자체(호모
        //   그래피)는 카메라가 돌아가도 안 바뀐다. bevScale() 의
        //   row->전방거리 / column->횡거리 유도도 intrinsic 과 소실점
        //   행(vanishing_y)만 쓰므로 순수 요(yaw) 회전에는 불변이다
        //   (yaw 는 지평선 행을 옮기지 않는다).
        //
        //   즉 buildCenterlinePath() 가 내는 좌표는 처음부터 "지금
        //   카메라가 보는 방향" 기준의 X전방/Y좌 였을 뿐이다. 거기에
        //   실측 pan 각(actual_pan_deg_)만큼 카메라 피벗을 중심으로
        //   회전만 더하면 base_link 기준이 된다 (8-b 절). 매 프레임
        //   경로가 pan 각을 따라 이렇게 계속 갱신되므로 tf2 로 행렬을
        //   다시 세우는 "진짜" 동적 BEV(픽셀 재추출)는 필요 없었다.
        //
        //   전에는 이 사실을 놓쳐 Idle 이 아닌 동안 통째로 건너뛰었고,
        //   그 결과 탐색+Holding+복귀 한 사이클(짧으면 수 초, 길면
        //   pan_hold_timeout_s 상한 없이 무한) 동안 /lane/center 가
        //   끊겨 하류에서 "경로가 생겼다 말았다" 로 보였다.
        // ================================================================

        enum class PanSearchState
        {
            Idle,       // pan = 0. 소실 감시만 한다
            Searching,  // 소실된 쪽으로 전진하며 재검출을 노린다
            Holding,    // 재검출됨. 직선 구간이 나올 때까지 그 각도를 유지
            Returning   // 0 으로 복귀 중
        };


        // 순서 있는 점열이 얼마나 휘었는가 [deg]. 판정 불가면 음수.
        //
        // 앞 절반의 현(chord)과 뒤 절반의 현이 이루는 각이다.
        //
        // BEV 좌표로 재도 된다. 호모그래피는 직선을 직선으로 보내므로
        // "휘었나 안 휘었나" 는 카메라가 돌아가 BEV 가정이 깨진
        // 상태에서도 그대로 읽힌다. 다만 각도의 크기는 보존되지
        // 않으므로(원근에 따라 늘거나 준다) 이 값은 세상의 도(度)가
        // 아니라 BEV 영상 위의 도다. 임계값은 실측으로 잡을 것.
        static double trackBendDeg(
            const std::vector<cv::Point2d> & pts);


        // pan 을 0 으로 되돌렸을 때 이 점들 중 몇 개가 여전히
        // 화면 안에 들어오는가. 계산 불가면 -1.
        //
        // ------------------------------------------------------------
        // 복귀 조건이 왜 "직선 구간" 이 아니라 이것인가
        //
        // 예전에는 도로가 펴지면 복귀했다. 그런데 "도로가 곧다" 와
        // "정면으로 돌아가도 그 선이 보인다" 는 다른 명제다.
        //
        // 실측: 정지 상태의 곧은 구간에서 왼쪽 흰선이 화면 밖에
        // 있었다. 도로는 곧으니 직선 판정은 통과하고, 복귀하면
        // 그 선은 여전히 안 보이니 즉시 재탐색이 걸렸다. 15초에
        // 9번 왕복했고 그동안 /lane/center 가 계속 끊겼다.
        // 직선 임계값을 아무리 조여도 멎지 않는 종류의 문제다.
        //
        // 그래서 목적을 직접 잰다 — 복귀해도 그 선을 다시 잡을
        // 수 있는가.
        //
        // ------------------------------------------------------------
        // 기하
        //
        // BEV -> 원본(undistort) 영상 역호모그래피로 추적점의 영상
        // 열 u 를 복원하면 영상각이 나온다 (오른쪽이 +):
        //
        //     a = atan((u - cx) / fx)
        //
        // 카메라가 pan 각 theta (왼쪽 +) 로 돌아가 있으므로 그 점의
        // 차량 기준 방위는 psi = theta - a 이고, pan = 0 일 때의
        // 영상각은
        //
        //     a0 = -psi = a - theta
        //
        // 이 값이 화면 반각(여유 제외) 안에 들면 정면에서도 보인다.
        //
        // 역호모그래피는 원본 영상과 BEV 사이의 순수 2D 사상이라
        // pan 때문에 깨진 평면 가정과 무관하게 정확하다. 깨지는
        // 것은 BEV 좌표의 "미터 해석" 이지 픽셀 대응이 아니다.
        // ------------------------------------------------------------
        int countVisibleAtZeroPan(
            const std::vector<cv::Point2d> & track_px,
            double pan_deg) const;


        // camera_pan_joint 실측 각도 수신 (50 Hz).
        void jointStateCallback(
            const sensor_msgs::msg::JointState::SharedPtr msg);


        // ================================================================
        // 측위 (map 프레임)
        //
        // CLAUDE.md §1 "측위 비의존" 원칙은 폐기했다. 경로를 map 프레임으로
        // 내보내고, /path/global 의 곡률 룩어헤드와 /perception/obstacles
        // (둘 다 map 프레임)로 pan 트리거를 판정하려면 자차의 map 위치가
        // 있어야 한다. 하나의 TF 조회로 세 용도를 전부 충당한다.
        // ================================================================

        struct VehiclePose
        {
            bool valid = false;

            double x_m = 0.0;

            double y_m = 0.0;

            double yaw_rad = 0.0;
        };


        // map <- base_link 를 frame_stamp(지금 처리 중인 영상의 시각)에서
        // 조회한다. 실패(TF 트리 미구성, 그 시각이 버퍼 밖 등)하면
        // valid=false — 호출측은 판단 근거가 없을 때 탐색을 막지 않는다
        // (fail-open, 이전 obstacleOnSide 와 같은 방침).
        VehiclePose lookupVehiclePose(const rclcpp::Time & frame_stamp) const;


        // /path/global 수신. latched 라 노드가 늦게 떠도 마지막 값을 받는다.
        void globalPathCallback(
            const kau_msgs::msg::KauPath::SharedPtr msg);


        // /perception/obstacles 수신. 판정에만 쓰므로 최신 한 장만 든다.
        void obstaclesCallback(
            const kau_msgs::msg::ObstacleCircleArray::SharedPtr msg);


        // 자차 위치에서 curvature_lookahead_m_ 이내 /path/global 최대
        // 곡률 [1/cm]. 판정 불가(측위/전역경로 없음)면 -1.
        //
        // /path/global 은 손으로 그려 한 번 latched 발행되는 정적 경로라
        // (kau_global_path README 1장), 자차가 지금 그 위 어디에 있는지를
        // 매 프레임 다시 찾아야 한다 — 최근접점을 구하고 거기서부터
        // 호길이로 lookahead_m 만큼 전진하며 지나는 구간들의
        // seg_kappa_max 중 최댓값을 취한다. seg_kappa_max 는 강체변환
        // 불변이므로(KauPath.msg) map 프레임 그대로 써도 된다.
        //
        // 최근접점 탐색은 `경로_형식.md` section 8.4 국소 window 추적이다.
        // 조각 하나의 최근접점은 8.1 (kau::bezier::nearestOnSeg 재사용).
        double curvatureAheadKappa(const VehiclePose & pose) const;


        // ------------------------------------------------------------
        // 최근접점 추적 상태 (경로_형식.md section 8.4)
        //
        // 매 프레임 전 구간을 훑으면(8.3 전역 탐색) 자차가 경로의 다른
        // 구간과 가까워지는 자리에서 반대편에 붙는다. 실측 lane_graph 의
        // 최소 자기근접 거리는 131.3 cm 이고, 규격은 그 상황에서 "횡오차
        // 155 cm 에서 반대편 구간 선택, s 971 cm 도약" 을 보고한다.
        //
        // 그래서 직전 해 주변 호길이 window 만 본다. 전역 탐색은 첫 프레임과
        // GATE 이탈이 연속 FAIL_LIMIT 회일 때만 쓴다.
        //
        // 상수는 규격 8.4 표의 값을 그대로 쓴다. kau_control 이 같은 값을
        // 갖지만 그 패키지를 의존으로 끌지 않는다 (bezier.hpp 사본을 두는
        // 것과 같은 이유).
        // ------------------------------------------------------------

        struct KappaTrack
        {
            std::size_t seg   = 0;      // 직전 해가 있던 조각
            double      u     = 0.0;    // 그 조각 안의 매개변수
            bool        valid = false;  // false 면 전역 탐색
            int         fails = 0;      // GATE 연속 이탈 횟수
        };

        static constexpr double kTrackFwdCm   = 100.0;  // 전방 window
        static constexpr double kTrackBackCm  = 50.0;   // 후방 window
        static constexpr double kTrackGateCm  = 250.0;  // 경로 이탈 판정
        static constexpr int    kTrackFailMax = 3;      // 연속 실패 -> 전역

        // const 메서드 안에서 갱신한다. 관측 결과 캐시라 논리적 상태가 아니다.
        mutable KappaTrack kappa_track_;


        // ================================================================
        // Pan 조준 (feedforward) — pan_aim_enable 일 때만
        //
        // /path/global 위 자차 최근접점에서 호길이로
        // pan_aim_lookahead_m 만큼 전진한 점이 base_link 기준 몇 도에
        // 있는가. 그 각도에 pan_aim_gain 을 곱한 값이 pan 목표각이다.
        // 판정 불가(측위/전역경로 없음)면 out_valid = false.
        //
        // 기존 updatePanSearch 상태기계가 "차선을 놓친 뒤" 찾아 도는
        // 되먹임이라면, 이쪽은 "도로가 휜 만큼 미리 돌리는" 앞먹임이다.
        //
        //   되먹임의 문제  검출 -> pan -> 검출 이 닫힌 고리라 발진한다
        //                  (§8 실측 15초에 9번 왕복). 탐색 지연도 크다
        //                  — 3프레임 소실 감지 후 램프+정착+스텝이라
        //                  커브가 끝날 무렵 도착한다.
        //   앞먹임의 이점  명령이 전역경로와 측위에서만 나오므로 검출을
        //                  거치지 않는다. 고리가 끊겨 발진 모드가 없다.
        //
        // 각도는 "그 점의 방위"(chord bearing)지 "그 지점 접선"이 아니다.
        // 접선을 쓰면 횡오차에 불변이지만, 그건 우리가 원하는 게 아니다
        // — 차가 실제로 20cm 밀려 있으면 차선도 그만큼 밀려 보이므로
        // 카메라도 따라가야 한다. 방위각은 곡률분과 횡오차분을 합쳐서
        // 낸다:
        //
        //     psi = d/(2R)  +  atan(e/d)      (R 곡률반경, e 횡오차)
        //
        // 측위 잡음이 e 에 섞여 들어오지만 AMCL 실측 지터 1.12cm 는
        // d=1m 에서 0.64도라 무시할 수준이다 (kau_localization README
        // "map -> odom 이 훨씬 안정적이다"). Cartographer 는 4.88cm /
        // 변화율 최대 167cm/s 라 허위 각속도가 96deg/s 까지 나오므로
        // 이 기능과 같이 쓰지 말 것 — 신호 대역(커브 진입 143deg/s)과
        // 겹쳐서 EMA 로도 못 가른다.
        //
        // 호길이 전진은 seg_length(구간 전체) + segLength(부분 구간
        // Gauss-Legendre) 이분탐색으로 정확히 잰다. curvatureAheadKappa
        // 의 u 균일 근사를 그대로 쓰면 안 된다 — 그쪽은 자기 주석대로
        // "트리거 판정용이지 제어에 쓰지 않는" 근사다.
        // ================================================================
        double panAimGoalDeg(
            const VehiclePose & pose,
            bool * out_valid) const;


        // ================================================================
        // Pan 조준 — 차선 기반 (pan_aim_source: "lane", 기본)
        //
        // 위 panAimGoalDeg 와 같은 "룩어헤드 점의 방위각" 이지만 근거가
        // /path/global + 측위가 아니라 **이 노드가 방금 만든 경로**다.
        // 측위도 TF 도 전역경로도 필요 없다.
        //
        // 성립하는 이유는 path.ctrl 이 이미 base_link 기준이기 때문이다
        // — buildCenterlinePath 가 camera_yaw_deg 만큼 pan 피벗 중심으로
        // 제어점을 회전시켜 놓는다. 즉 여기서 잰 방위각은 "카메라가 보는
        // 방향 기준" 이 아니라 **차체 기준 절대각**이다. 카메라를 10도
        // 돌려도 차선이 그대로면 다음 프레임에 나오는 값은 같다. 각이
        // 적분되지 않으므로 §8 되먹임(검출->pan->검출)의 발산 모드가
        // 생기지 않는다.
        //
        // ★ 다만 **완전히 열린 고리는 아니다.** 그 회전 보정에 쓰는 각도는
        //   실기에서 서보 인코더가 없어 명령의 에코다. 서보 추종 지연
        //   delta = theta_cmd - theta_act 만큼 경로가 과회전되고, 그것이
        //   방위각에 그대로 더해져 다음 명령을 키운다:
        //
        //     카메라가 실제로 본 각   psi - theta_act
        //     +theta_cmd 회전 보정 -> psi + (theta_cmd - theta_act)
        //
        //   DC 루프게인이 1이면 이 오차가 안 죽는다. 그래서 이 소스에서는
        //   pan_aim_gain 을 1 미만으로 둔다 (config 0.7). 전역경로 소스는
        //   고리 자체가 없어 1.0 이 안전했다 — 소스를 바꿀 때 gain 도
        //   같이 봐야 하는 이유다.
        //
        // 룩어헤드 거리는 근거 길이로 자른다. path.valid_length_cm 은
        // "근거 점이 끊기기 전까지의 호길이" 이고 그 뒤는 외삽이므로
        // (buildCenterlinePath 8절), 가림이 생기면 이 값이 저절로 줄어든다.
        // 즉 **가려지지 않은 데까지만 보고 조준한다** 가 클램프 한 줄로
        // 구현된다. 따로 가림 검출을 두지 않는다.
        //
        // 거리 기준점 주의: valid_length_cm 도 여기 이분탐색도 **경로
        // 시작점**(카메라 앞 x_base_cm, 약 32cm) 기준이다. 반면
        // pan_aim_lookahead_m 은 **자차 기준** 거리로 읽히는 이름이라,
        // 전역경로 소스와 의미를 맞추려고 x_base_cm 을 빼고 들어간다.
        //
        // 무효 조건 (out_valid = false, 호출측이 홀드/폴백을 정한다):
        //   - path.valid 가 아니다 (게이트 기각 포함)
        //   - valid_length_cm < pan_aim_min_evidence_cm
        //   - 조준점이 자차 원점과 겹쳐 방위가 정의되지 않는다
        // ================================================================
        double panAimGoalDegFromLane(
            const LanePath & path,
            bool * out_valid) const;


        // 조준각의 상한 [deg]. pan_max_deg 와 "근거가 화면에 남는
        // 각도" 중 작은 쪽이다.
        //
        // 후자는 수평 반각 atan(cx/fx) 에서 pan_aim_fov_margin_deg 를
        // 뺀 값이다 (countVisibleAtZeroPan 과 같은 기하). 너무 돌려서
        // 근거를 잃고 -> 조준 무효 -> 복귀 -> 다시 보임 이 되풀이되는
        // 것을 막는 안전턱이다.
        //
        // 실측 fx 201.4 / cx 240 이면 반각 50도라 여유 47도 > pan_max
        // 30도 로 **지금 광학에서는 걸리지 않는다.** pan_max_deg 를
        // 올리거나 렌즈를 바꿀 때 살아나는 쪽이다. intrinsic 이 없으면
        // pan_max_deg 를 그대로 반환한다.
        double panAimLimitDeg() const;


        // 소실된 쪽 dir 방향, 전방 obstacle_ahead_max_m_ 이내에 장애물이
        // 있는가. dir 은 pan_search_dir_ 와 같은 규약 (+1 왼쪽 / -1 오른쪽).
        //
        // 차선이 화면에서 사라지는 이유는 두 가지다. 코너를 돌면서 시야
        // 밖으로 나갔거나, 앞의 장애물에 가렸거나. 앞의 경우는 고개를
        // 돌리면 다시 보이지만, 뒤의 경우는 아무리 돌려도 안 보인다 —
        // 가린 물체가 같이 따라오기 때문이다. 그래도 탐색을 걸면 상한까지
        // 헛돌고 그동안 경로만 끊긴다.
        //
        // /perception/obstacles(map 프레임, kau_object_detection)의 원을
        // 자차 기준(전방 +x, 좌측 +y)으로 옮겨 판정한다. 그 노드는 이제
        // Gazebo 참값이 아니라 tf2(map<-lidar_link)로 원을 앉히므로 실기
        // 에서도 쓸 수 있다 (laser_scan_clusterer_node.cpp 참고). STATUS_OK
        // 가 아니거나 오래됐으면 false — fail-open.
        bool obstacleAheadOnSide(
            int dir,
            const VehiclePose & pose,
            const rclcpp::Time & frame_stamp) const;


        // 절대각 [deg] 을 pan_max_deg_ 로 클램프해 /camera/pan 에
        // rad 로 발행하고 pan_cmd_deg_ 를 갱신한다.
        //
        // 한때 목표/발행을 분리해 매 프레임 각속도 제한으로 다가가는
        // 연속 램프를 썼지만, 실기에서 "꺾다 말다" 하는 움직임이
        // 나와 이산 스텝으로 되돌렸다.
        // 목표각만 세운다. 실제 발행은 publishPanRamped 가 한다.
        void commandPan(double goal_deg);

        // 목표각을 향해 pan_rate_deg_s_ 로 한 프레임분만 다가가
        // 발행한다. 매 프레임 무조건 호출된다 (목표가 안 바뀌어도
        // 램프가 진행 중일 수 있으므로).
        void publishPanRamped(const rclcpp::Time & frame_stamp);


        // 좌/우 원본 검출 상태를 보고 위 상태기계를 한 프레임 굴린다.
        // 반드시 흰선 검증(16-a) 뒤, 소실 복원(16-c) 과 무관한 시점에
        // 부른다 (복원선은 found_count 가 0 이라 어느 쪽이든 같다).
        // frame_stamp 는 지금 처리 중인 영상의 시각이다. 각속도
        // 제한의 dt 와 Holding 경과시간, 장애물 판정의 scan 나이를
        // 전부 이 시계로 잰다. node clock 을 쓰면 use_sim_time 설정과
        // RTF 에 따라 전부 어긋난다.
        //
        // lane_path 는 이 프레임의 (EMA 까지 끝난) 중심선 경로다.
        // pan_aim_source: "lane" 일 때 조준 근거가 된다. 그래서 이
        // 호출은 16-c-2 가 아니라 **16-d-2(robustifyPath) 뒤**에 있다
        // — 직전 프레임 경로를 쓰면 14Hz 에서 70ms 지연이고, v_max
        // 요구 각속도 143deg/s 기준 10도라 못 쓴다.
        void updatePanSearch(
            const LaneDetectionResult & left,
            const LaneDetectionResult & yellow,
            const LaneDetectionResult & right,
            const LanePath & lane_path,
            const rclcpp::Time & frame_stamp);


        // ================================================================
        // 시간축 강건화 (EMA)
        //
        // 게이트는 프레임 하나만 보고 통과/기각을 정한다. 임계값을
        // 조이면 멀쩡한 차선이 떨어져 나가고, 풀면 도로 밖 노란
        // 물체(원경의 그림 등)가 통과한다. 어느 쪽이든 하류
        // state machine 의 fail-safe 판정이 흔들린다.
        //
        // 프레임 하나로 정할 수 없는 문제이므로 시간축을 쓴다.
        // 게이트를 통과한 관측을 EMA 추정치에 섞되,
        //
        //   1. 섞는 양(alpha)을 그 프레임의 confidence 에 비례시킨다.
        //      믿기 힘든 프레임은 추정치를 조금밖에 못 움직인다.
        //
        //   2. 추정치에서 path_ema_gate_cm_ 이상 벗어난 관측은
        //      아예 섞지 않고, 직전 추정치를 그대로 내보내면서
        //      confidence 만 떨어뜨린다. 하류는 "경로는 계속 오는데
        //      신뢰도가 미끄러진다" 를 보고 스스로 판단할 수 있다.
        //
        //   3. 같은 주장이 path_ema_relock_frames_ 번 연속 오면
        //      노이즈가 아니라 실제 변화로 보고 통째로 재잠금한다.
        //      (그러지 않으면 진짜 차선 변화에 영원히 눈감는다)
        //
        // 자기운동 보정은 하지 않는다. 이 노드는 측위 비의존이
        // 대전제라 odom/TF 를 볼 수 없다. 차선을 따라가는 동안
        // base_link 기준 경로는 거의 불변이라 편향이 작지만,
        // 고속에서는 추정치가 뒤로 끌리는 지연이 생긴다.
        // ================================================================

        // 제어점 열 사이의 평균 거리 [cm]. 크기가 다르면 음수.
        static double pathDeviationCm(
            const kau::bezier::Ctrl & a,
            const kau::bezier::Ctrl & b);


        // ctrl 이 바뀐 뒤 진단값(길이/곡률/cte/heading)을 다시 맞춘다.
        // EMA 로 섞은 제어점은 관측 당시의 진단값과 더 이상 짝이
        // 맞지 않는다. valid_length 는 하류가 LookAhead 를 자르는
        // 근거이므로 특히 그대로 두면 안 된다.
        static void refreshPathMetrics(LanePath & path);


        // 관측 경로를 EMA 추정치와 합쳐 발행할 경로로 바꾼다.
        // path.valid 가 false 면 관측 없음으로 보고 상태만 늙힌다.
        // 수정
        void robustifyPath(LanePath & path, const rclcpp::Time & frame_stamp);

        void stepEmaCtrlToward(const kau::bezier::Ctrl & target, double max_step_cm);


        // 발행 게이트 (docs/경로_형식.md 와 무관한 안전장치).
        // 통과면 0, 아니면 기각 코드.
        int pathGate(
            const LanePath & path,
            double sx,
            double camera_yaw_deg) const;

        static const char * gateName(int code);

        // cv::putText 는 CJK 글리프가 없어 물음표로 나온다.
        // debug 화면용 ASCII 이름.
        static const char * gateNameAscii(int code);

        // lane_width_cm + 카메라 기하 -> lane_width_px_ 재계산.
        // CameraInfo 수신 시와 BEV 기하 변경 시 호출한다.
        void refreshDerivedScale();


        // 제어점 -> KauPath 적재 후 발행 (docs/경로_형식.md 9.5).
        // 게이트를 통과한 경로만 발행한다.
        // pub 이 null 이면 lane_path_publisher_(/lane/center) 로 낸다.
        // 좌/우 흰선 경로는 같은 함수에 publisher 만 바꿔 넘긴다 —
        // map 변환 / 단위 / 필드 채우기가 셋 다 같아야 하기 때문이다.
        void publishLanePath(
            const LanePath & path,
            const rclcpp::Time & stamp,
            const rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr &
                pub = nullptr);


        // 차선 한 줄(추적점 하나)을 quintic Bezier 경로로 적합한다.
        //
        // buildCenterlinePath 의 3-b ~ 8 절과 같은 식이다. 다른 것은
        // 입력이 "세 차선을 합성한 중심선 후보" 가 아니라 추적점
        // 하나라는 것뿐이라, 1~3 절(후보 합성 / spine 선정 / 이탈
        // 제거)이 통째로 필요 없다.
        //
        // 좌/우 흰선을 /lane/left, /lane/right 로 내보내기 위한 것이다
        // (측위 없이 corridor 경계를 하류에 넘기는 용도).
        LanePath buildEdgePath(
            const LaneDetectionResult & lane,
            int bev_width,
            int bev_height,
            double camera_yaw_deg);


        // BEV 축척 [cm/px]. 세로는 intrinsic + 소실점에서 유도.
        bool bevScale(
            double & sx,
            double & sy,
            double & x_base_cm) const;


        // 차선 중심선 -> quintic Bezier 제어점 (공통 경로 형식).
        //
        // camera_yaw_deg 는 지금 카메라가 차체 정면에서 얼마나 돌아가
        // 있는가 (+deg 왼쪽, pan_search_dir_/actual_pan_deg_ 와 같은
        // 규약). 0 이 아니면 8-b 절에서 그만큼 회전 보정한 뒤 낸다 —
        // pan 중에도 경로를 계속 짓기 위함 (§8 절 헤더 주석 참고).
        LanePath buildCenterlinePath(
            const LaneDetectionResult & left,
            const LaneDetectionResult & yellow,
            const LaneDetectionResult & right,
            int bev_width,
            int bev_height,
            double camera_yaw_deg,
            std::vector<cv::Point2d> & center_pts_px);


        // 발행한 Bezier 를 BEV 위에 되돌려 그린다 (시각화 전용).
        //
        // path.ctrl 은 base_link 기준(buildCenterlinePath 의 camera_yaw_deg
        // 회전 보정 후)이지만 이 BEV 이미지는 "지금 카메라가 보는 방향"
        // 기준이므로, camera_yaw_deg 로 역회전해 그린다 — 안 그러면 pan
        // 중에 곡선이 실제 차선과 어긋나 보인다.
        void drawPathOverlay(
            const LanePath & path,
            double camera_yaw_deg,
            cv::Mat & image);


        // 메인 파이프라인.
        void imageCallback(
            const sensor_msgs::msg::Image::SharedPtr msg);


        // ================================================================
        // Subscribers
        // ================================================================

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
            image_subscriber_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
            camera_info_subscriber_;

        // camera_pan_joint 실측 각도. 실기 드라이버는 서보 인코더가
        // 없어 마지막 명령각을 되쏘고, 시뮬은 Gazebo 실측 관절각이다.
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
            joint_state_subscriber_;

        // 전역 경로. latched, map 프레임. 곡률 룩어헤드에 쓴다.
        rclcpp::Subscription<kau_msgs::msg::KauPath>::SharedPtr
            global_path_subscriber_;

        // 장애물 가림 판정용. map 프레임, 실기/시뮬 공통.
        rclcpp::Subscription<kau_msgs::msg::ObstacleCircleArray>::SharedPtr
            obstacles_subscriber_;


        // ================================================================
        // Publishers
        // ================================================================

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            undistorted_image_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            bev_image_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            debug_image_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
            roi_overlay_publisher_;

        // 검출 상태 진단. 실패 프레임 수집 스크립트가 이걸 보고 판정한다.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
            status_publisher_;

        // 이 노드의 최종 산출물. /lane/center, RELIABLE depth 1 (9.9)
        rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr
            lane_path_publisher_;

        // 좌/우 흰선 경로. /lane/center 와 같은 KauPath 형식이고
        // frame_id 도 path_frame_id_ 를 따른다. 하류(local planner)가
        // 이 둘을 corridor 물리 경계로 쓴다 — 맵/측위를 안 쓰기 위한
        // 대체 경로다.
        rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr
            lane_left_publisher_;

        rclcpp::Publisher<kau_msgs::msg::KauPath>::SharedPtr
            lane_right_publisher_;

        // RViz 전용. BEST_EFFORT. 제어 경로는 여기 의존하지 않는다.
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
            viz_path_publisher_;

        // Pan 명령. physicar_driver_node 의 apply_pan() 이 절대각
        // [rad] 으로 받아 ±30 deg 로 다시 클램프한다.
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
            camera_pan_publisher_;

        // Tilt 명령. pan 과 달리 제어량이 아니라 **기동 시 한 번 세우는
        // 고정 자세**다. bev_vanishing_y 가 이 각도를 전제하므로(수평
        // 대비 9 px) 누가 안 세워 주면 BEV 기하가 그만큼 어긋난다.
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
            camera_tilt_publisher_;

        // 기동 직후 몇 번 반복 발행하고 스스로 멈추는 타이머.
        // 한 번만 쏘면 드라이버/브리지가 늦게 뜰 때 유실되고,
        // 계속 쏘면 웹UI 틸트 슬라이더를 매초 되돌려 버린다.
        rclcpp::TimerBase::SharedPtr camera_tilt_timer_;

        int camera_tilt_ticks_left_ = 0;


        // 기동 시 세울 tilt 각 [deg]. **부호는 /camera/tilt 규약 그대로
        // + 가 아래**다 (camera_tilt_joint axis = +Y, URDF/SDF 공통).
        // bev_vanishing_y 유도에 쓰는 광학 pitch 부호와는 반대이니
        // 헷갈리지 말 것 — 그쪽은 아래가 음수다.
        double camera_tilt_deg_ = 0.0;

        bool camera_tilt_enable_ = false;

        double camera_tilt_repeat_s_ = 0.0;


        // 기동 tilt 를 한 번 발행한다 (타이머 콜백).
        void publishCameraTilt();


        // ================================================================
        // BEV Parameters
        // ================================================================

        double bev_src_center_x_;

        double bev_src_top_y_;

        double bev_src_bottom_y_;

        double bev_vanishing_y_;

        // 파라미터가 아니라 소실점에서 계산되는 값
        double bev_src_top_width_ = 0.0;

        double bev_src_bottom_width_;

        double bev_dst_margin_ratio_;

        int bev_out_width_;

        int bev_out_height_;

        int window_margin_;

        int min_pixels_;

        // 방향성 추적: 한 스텝 전진 거리 [BEV px].
        // 코너 추종 능력을 지배하는 값이다.
        double track_step_px_;

        // 방향성 추적: 한 스텝에서 허용하는 최대 방향 변화 [deg]
        double track_max_turn_deg_;

        // 방향성 추적: 최대 스텝 수 (곡선은 직선보다 스텝이 더 필요)
        int track_max_steps_;

        // 방향성 추적: 연속으로 비어도 관성 주행할 스텝 수 (점선 대응)
        int track_max_miss_;

        // 흰선이 노란선 기준으로 제 쪽에 있는지 판정 [lane_width_px 비율].
        // 이보다 가까우면 반대쪽 선을 문 것으로 보고 버린다.
        double lane_side_min_ratio_;

        // 흰선 회랑 [lane_width 비율]. 이 밖으로 나가면 그 지점에서 절단.
        double lane_corridor_lo_;

        double lane_corridor_hi_;

        // 중심선 후보 이탈 판정 [lane_width_px 비율].
        // 기준선에서 이만큼 이상 떨어진 후보는 다른 선을 문 것으로 본다.
        double center_outlier_ratio_;

        std::vector<int64_t> yellow_hls_lo_;

        std::vector<int64_t> yellow_hls_hi_;

        std::vector<int64_t> white_hls_lo_;

        std::vector<int64_t> white_hls_hi_;

        // 차로 폭 [BEV px]. 파라미터가 아니라 lane_width_cm 과
        // 카메라 기하에서 유도한 값 (refreshDerivedScale 참고)
        double lane_width_px_;

        // 히스토그램 밴드 높이 (BEV 하단에서 차지하는 비율)
        double histogram_band_ratio_;

        bool publish_roi_overlay_;


        // ================================================================
        // 경로 발행 파라미터 (docs/경로_형식.md)
        // ================================================================

        std::string path_frame_id_;

        bool publish_path_;

        double lane_width_cm_;

        double camera_height_cm_;

        double path_x_offset_cm_;

        double path_y_offset_cm_;

        bool draw_path_overlay_;


        // ================================================================
        // 중앙선 대비 자차 위치 (진단 + identifyLine prior)
        //
        // 차로 번호 개념은 없다. 남은 건 "중앙선에서 자차가 얼마나
        // 떨어져 있는가" 라는 스칼라값 하나뿐이다.
        // ================================================================

        // 근거가 없어도 직전 판정을 유지할 프레임 수
        int center_hold_frames_;

        // 판정 이력
        CenterFix center_fix_;

        int center_fix_miss_ = 0;

        // 확실한 근거(노란선 또는 흰선 두 개)로 마지막에 잰
        // 중앙선 대비 자차 횡거리 [cm]. identifyLine 의 기준점.
        double center_prior_offset_cm_ = 0.0;

        // 위 값이 몇 프레임 전 것인지. 오래되면 안 쓴다.
        int center_prior_age_ = 1000000;


        // ================================================================
        // 발행 게이트
        // ================================================================

        bool path_gate_enable_;

        int min_path_windows_;

        double wheelbase_cm_;

        double max_steer_deg_;

        double max_lateral_cm_;


        // ================================================================
        // 시간축 강건화 (EMA) — 파라미터와 상태
        // ================================================================

        bool path_ema_enable_;

        // confidence = 1 일 때의 혼합 비율. 작을수록 무겁고 느리다.
        double path_ema_alpha_;

        // 추정치에서 이만큼(제어점 평균거리) 벗어난 관측은 섞지 않는다.
        // 0 이하면 게이트 없이 순수 EMA.
        //
        // **공칭 주기(14 Hz) 기준값이다.** 프레임이 느려지면 그만큼
        // 넓혀서 쓴다 (robustifyPath 참고) — 안 그러면 느린 프레임에서
        // 멀쩡한 관측이 전부 걷어차여 추정치가 얼어붙는다.
        double path_ema_gate_cm_;

        // 위 게이트를 dt 에 비례해 넓힐 때의 배율 상한.
        // 파이프라인이 멎었을 때 게이트가 통째로 무력화되는 걸 막는다.
        double path_ema_gate_dt_scale_max_;

        // 기각이 이만큼 연속되면 실제 변화로 보고 재잠금한다.
        int path_ema_relock_frames_;

        // 관측이 이만큼 연속으로 없으면 추정치를 폐기한다.
        int path_ema_reset_frames_;


        kau::bezier::Ctrl path_ema_ctrl_;

        bool path_ema_valid_ = false;

        double conf_ema_ = 0.0;

        double valid_len_ema_ = 0.0;

        int path_ema_reject_streak_ = 0;

        // 직전 프레임의 관측-추정치 편차 [cm]. 진단 전용
        // (path_ema_gate_cm 을 실측으로 잡는 근거가 된다).
        double last_path_dev_cm_ = -1.0;

        // 직전 프레임에 실제로 적용된 게이트 [cm]. dt 로 넓어진
        // 뒤의 값이라, edev 와 나란히 봐야 기각 여부가 설명된다.
        double last_path_gate_cm_ = -1.0;

        // 공칭 프레임 주기 [s]. 스탬프가 없거나 튄 프레임의 대체값
        // 이자, 게이트를 dt 로 넓힐 때의 기준이다.
        static constexpr double kNominalFrameSec = 1.0 / 14.0;

        int ema_miss_streak_ = 0;


        // ================================================================
        // Pan 트리거 판정 — 곡률(경로+시야) + 장애물 (map 프레임)
        //
        // 예전에는 /scan_filtered 를 직접 스캔해 "그 방향 부채꼴 안에
        // 반사가 있는가" 로 가림을 판정했다. 이제는 자차의 map 위치를
        // 알므로 더 직접적인 두 근거로 바꾼다.
        //
        //   곡률   둘 중 하나라도 급커브를 가리키면 참 (OR).
        //            경로 기반 /path/global 위 전방 curvature_lookahead_m_
        //                      안의 최대 곡률. 측위 + 전역경로 수신 필요.
        //            시야 기반 가려지지 않은 쪽(노란 중앙선 — 이 Idle
        //                      분기는 yellow 가 보인다는 걸 이미 전제
        //                      한다)의 꺾임(trackBendDeg). 측위 없이도
        //                      항상 계산된다.
        //          가려진 쪽(놓친 흰선)으로는 절대 판정하지 않는다 —
        //          안 보이는 선의 곡률은 잴 수 없다.
        //   장애물 /perception/obstacles(map 프레임) 중 소실된 쪽
        //          전방 obstacle_ahead_max_m_ 안에 원이 있는가.
        //
        // 두 판정이 엇갈리면(급커브 구간인데 그 안에 장애물 원도 있음)
        // 곡률을 우선한다 — 트랙 경계벽이 장애물로 오검출됐을 여지가
        // 더 크기 때문이다 (updatePanSearch 구현부 참고).
        // ================================================================

        bool obstacle_pan_block_enable_;

        // 자차 위치에서 /path/global 위 전방으로 이만큼 [m] 살펴
        // 최대 곡률을 잰다.
        double curvature_lookahead_m_;

        // 위에서 잰 최대 곡률 [1/cm] 이 이 이상이면 "그 방향은 원래
        // 급커브" 로 보고 장애물 판정 없이 pan 탐색을 진행한다.
        // 근거: 실측 전 초안값. 튜닝 필요.
        double curve_ahead_kappa_thresh_;

        // 가려지지 않은 쪽(노란 중앙선)의 꺾임 [deg] 이 이 이상이면
        // 역시 "그 방향은 원래 급커브" 로 본다. 측위/전역경로가 아직
        // 없을 때(기동 초반 등)도 동작하는 근거다. 근거: 실측 전
        // 초안값. 튜닝 필요.
        double curve_ahead_bend_deg_thresh_;

        // 소실된 쪽 전방 이 거리 [m] 안에 장애물 원이 있으면 가림으로
        // 본다 (곡률이 급커브를 가리키지 않을 때만 적용).
        double obstacle_ahead_max_m_;

        // 장애물 원이 차량 종축에서 이 횡거리 [m] 이내여야 우리 차로
        // 안으로 본다. 이보다 멀면 트랙 경계벽 등으로 보고 무시한다.
        double obstacle_lane_lateral_m_;

        // /perception/obstacles 가 이보다 오래되면 판단하지 않는다 [s].
        double obstacle_scan_timeout_s_;


        kau_msgs::msg::KauPath::SharedPtr global_path_;

        kau_msgs::msg::ObstacleCircleArray::SharedPtr latest_obstacles_;


        // ================================================================
        // 측위 (map 프레임)
        // ================================================================

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;


        // ================================================================
        // Pan 탐색 상태
        // ================================================================

        bool pan_search_enable_;

        // 탐색 첫 목표각 [deg].
        //
        // 차선이 화면에서 빠지는 상황은 대개 코너라 몇 도로는
        // 다시 안 들어온다. 3도씩 기어가면 재검출까지 프레임을
        // 낭비하고 그동안 경로도 낡는다. 그래서 첫 목표는 크게
        // 잡는다 — 다만 **도달은 램프로** 한다 (예전에는 이
        // 각도로 한 프레임에 점프했다).
        double pan_start_deg_;

        // 시작 각도에서 못 찾았을 때의 한 스텝 회전량 [deg].
        double pan_step_deg_;

        double pan_max_deg_;

        // 목표각으로 다가가는 각속도 상한 [deg/s].
        //
        // 예전에는 목표각을 그대로 한 프레임에 발행했다. 그러면
        // 서보는 매번 "최대 슬루로 쫓아가는" 상태가 되고, 카메라가
        // 도는 동안 찍힌 영상이 계속 들어온다. 그 영상으로 지은
        // 경로를 회전 보정할 때 쓰는 각도(actual_pan_deg_)는 영상
        // 시각이 아니라 처리 시각의 값이라, 슬루가 빠를수록 그
        // 어긋남이 커진다 (77cm 경로 기준 Δθ 3도 = 끝단 4.0cm,
        // 15도 = 19.9cm).
        //
        // 각속도를 묶으면 그 어긋남의 상한도 같이 묶인다:
        //   끝단 오차 <= L * pan_rate_deg_s_ * (파이프라인 지연)
        // 30 deg/s, 지연 50ms 면 1.5도 -> 77cm 에서 2.0cm 다.
        // 즉 램프는 "부드러움" 뿐 아니라 **회전 보정 오차의
        // 상한**을 준다 — 지금 방식보다 오히려 정확해진다.
        double pan_rate_deg_s_;

        // |실측 - 목표| 가 이 안이면 정착으로 보고 다음 스텝으로 간다.
        double pan_settle_tol_deg_;

        int pan_trigger_miss_frames_;

        // 명령 부호 뒤집개. 카메라가 명령과 반대로 돌면 -1.0.
        double pan_sign_;

        // 재검출 직후 추가로 더 돌릴 각도 [deg].
        //
        // 탐색은 "겨우 보이기 시작한" 지점에서 멈춘다. 그 각도에서는
        // 찾은 선이 화면 가장자리에 걸쳐 있어, 차가 조금만 움직여도
        // 다시 빠진다. 한 번 더 꺾어 안쪽으로 당겨 놓는다.
        // pan_max_deg_ 는 commandPan 이 그대로 지킨다.
        double pan_overshoot_deg_;


        // 재검출 후 복귀 조건 — 직선 구간
        //
        // 찾자마자 복귀하면 코너 한복판에서 원위치로 돌아가 그
        // 선을 곧바로 다시 놓친다. pan 이 왕복하며 그동안 경로가
        // 계속 끊긴다. 그래서 재검출 뒤에는 각도를 유지한 채
        // (Holding) 도로가 다시 펴질 때까지 기다렸다 복귀한다.
        // 화면 가장자리 여유 [deg]. 반각에서 이만큼 뺀 범위만
        // "보인다" 로 친다. 가장자리에 겨우 걸친 선은 차가 조금만
        // 움직여도 다시 빠지기 때문이다.
        double pan_return_fov_margin_deg_;

        int pan_return_confirm_frames_;

        // 직선 판정에 필요한 최소 추적점 수.
        //
        // 짧은 점열은 어떤 코너에서도 직선으로 보인다 (구현부
        // 주석 참고). 근거가 모자라면 "직선 아님" 이 아니라
        // "판정 불가" 로 다뤄 Holding 을 유지한다.
        int pan_return_min_points_;

        // 직선이 끝내 안 나올 때의 탈출구 [s].
        //
        // 0 이하면 무한 대기 — 직선을 만날 때까지 복귀하지 않는다
        // (기본값). pan 중에도 경로는 계속 짓지만(§8, buildCenterlinePath
        // camera_yaw_deg 보정), Holding 이 오래갈수록 카메라가 정면이
        // 아닌 채로 보는 시야만 근거로 삼는 시간이 길어진다. 그게
        // 곤란해지면 양수로 바꿔 상한을 준다.
        double pan_hold_timeout_s_;

        // pan 회전축(camera_pan_joint)이 base_link 원점에서 떨어진
        // 거리 [cm]. X 전방 / Y 좌.
        //
        // buildCenterlinePath 가 camera_yaw_deg 로 회전 보정할 때 이
        // 피벗을 원점으로 쓴다. 0,0 이면 회전축이 base_link 원점과
        // 같다고 가정하는 것과 같다 — URDF/SDF 실측으로는 약 (5, 0)
        // cm 이지만, pan=0 에서는 이 값이 얼마든 결과에 영향이 없으므로
        // (회전량 0) 정확한 값을 몰라도 기존 동작은 그대로다. 카메라
        // 마운트 재실측(camera_height_cm 갱신) 시 같이 넣을 것.
        double pan_pivot_offset_x_cm_;

        double pan_pivot_offset_y_cm_;


        // ------------------------------------------------------------
        // Pan 조준 (feedforward). panAimGoalDeg 선언부 주석 참고.
        //
        // 셋 다 매 프레임 get_parameter 로 다시 읽는다 (bev 파라미터와
        // 같은 idiom). pan_search_enable 과 달리 ros2 param set 으로
        // 주행 중에 끄고 켤 수 있다 — 되돌릴 때 재빌드가 필요 없어야
        // 하기 때문이다.
        // ------------------------------------------------------------

        // false 면 이 블록이 통째로 꺼지고 기존 탐색 상태기계가
        // 그대로 동작한다.
        bool pan_aim_enable_;

        // 최근접점에서 호길이로 이만큼 [m] 전진한 점을 조준한다.
        double pan_aim_lookahead_m_;

        // 조준 방위각에 곱하는 배수.
        double pan_aim_gain_;

        // 진단용 (status 의 paim / pav).
        double last_pan_aim_deg_ = 0.0;

        bool last_pan_aim_valid_ = false;


        // 조준 근거. "lane" (기본) 또는 "global".
        //
        //   lane    이 노드가 방금 만든 중심선 경로 (측위 불필요)
        //   global  /path/global + 측위 (예전 동작)
        //
        // 매 프레임 다시 읽으므로 실차에서 주행 중 A/B 비교가 된다.
        // 아는 값이 아니면 "lane" 으로 보고 5초에 한 번 경고한다.
        std::string pan_aim_source_;

        // 조준하려면 근거(valid_length_cm)가 최소 이만큼 [cm] 있어야
        // 한다. 짧은 점열은 어떤 코너에서도 직선으로 보이므로
        // (CLAUDE.md §8 "짧은 점열로는 판정하지 않는다"), 여기서도
        // 같은 이유로 하한을 둔다. lane 소스에서만 쓴다.
        double pan_aim_min_evidence_cm_;

        // 조준각이 직전 목표에서 이만큼 [deg] 넘게 달라졌을 때만
        // 목표를 다시 세운다. 램프가 매 프레임 다시 출발하는 것을
        // 막는다 (그게 "꺾다 말다" 로 보인다).
        double pan_aim_deadband_deg_;

        // 근거가 모자란 프레임을 몇 장까지 직전 조준각으로 버틸지.
        //
        // 여기서 0 으로 되돌리면 안 된다 — "안 보임 -> 정면 복귀 ->
        // 보임 -> 다시 조준" 이 §8 실측 왕복(15초 9회)의 형태 그대로다.
        // 이 장수를 넘기면 조준을 포기하고 §8 탐색 상태기계로 넘긴다
        // (차선이 통째로 사라진 상황은 탐색이 맡는 게 맞다).
        int pan_aim_hold_frames_;

        // panAimLimitDeg 의 FOV 여유 [deg]. 선언부 주석 참고.
        double pan_aim_fov_margin_deg_;

        // 근거 부족으로 직전 조준각을 유지하고 있는 연속 프레임 수.
        // 근거가 돌아오면 0 으로 끊는다. status 의 pahold.
        int pan_aim_hold_streak_ = 0;

        // 데드밴드 비교 기준이 되는 "마지막으로 세운 목표각".
        // pan_goal_deg_ 를 직접 보면 안 된다 — 그건 탐색 상태기계도
        // 같이 쓰는 값이라 폴백을 거친 뒤 기준이 오염된다.
        double pan_aim_goal_deg_ = 0.0;

        bool pan_aim_goal_init_ = false;

        // 재검출 인정에 필요한 최소 창 수 (Searching -> Holding).
        //
        // 예전에는 found_count > 0, 즉 창 하나였다. min_pixels 만큼의
        // 흰 픽셀이 회랑 안에 한 번 잡히면 그 각도에서 멈췄다는 뜻인데
        // 문제가 셋이었다.
        //
        //  (a) detectLaneSlidingWindow 는 found_count >= MIN_VALID_WINDOWS
        //      (3) 를 넘겨야 detected/valid 로 친다. pan 정지만 그보다
        //      느슨해서 "차선으로 인정도 안 되는 검출" 에 멈췄다.
        //  (b) 탐색 시작은 pan_trigger_miss_frames(3) 프레임 연속 소실이
        //      필요한데 정지는 1프레임이었다. 시작은 어렵고 멈추기는
        //      쉬운 반대 방향 히스테리시스라 반짝임 하나에 중단됐다.
        //  (c) 창 하나는 선 끄트머리가 화면 가장자리에 겨우 걸친
        //      상태다. 거기서 Holding 에 들어가면 경로 품질이
        //      회복되지 않은 채 유지된다.
        //
        // track_step_px 25 기준 6창은 호길이 약 150 px 이다.
        int pan_found_min_windows_;

        // 위 조건이 연속으로 성립해야 하는 프레임 수.
        // pan_trigger_miss_frames 와 대칭을 맞추기 위한 것이다.
        int pan_found_confirm_frames_;


        PanSearchState pan_state_ = PanSearchState::Idle;

        // 탐색 회전 방향. +1 왼쪽 / -1 오른쪽 / 0 탐색 중 아님.
        // camera_pan_joint 는 axis +Z 라 +rad 이 왼쪽이다.
        int pan_search_dir_ = 0;

        // 상태기계가 세운 목표각 [deg]. 발행값은 여기로 램프한다.
        double pan_goal_deg_ = 0.0;

        // 실제로 /camera/pan 에 발행한 각도 [deg]
        double pan_cmd_deg_ = 0.0;

        // 램프 dt 계산용. 노드 클록이 아니라 **영상 프레임 스탬프**
        // 로 잰다 (use_sim_time=false 인데 센서가 sim time 을 달고
        // 오는 함정 회피 — CLAUDE.md §9 시계 주의).
        rclcpp::Time pan_ramp_last_stamp_;

        bool pan_ramp_time_init_ = false;

        // /joint_states 가 알려준 **가장 최근** 각도 [deg].
        //
        // 이 값은 "서보가 지금 명령에 도달했는가"(servo_ok) 판정
        // 전용이다. 경로 회전 보정에는 쓰면 안 된다 — 그건 영상이
        // 찍힌 시각의 각도여야 하므로 panAngleAt() 을 쓴다.
        double actual_pan_deg_ = 0.0;

        bool joint_state_received_ = false;

        // ------------------------------------------------------------
        // pan 각도 이력 — 영상 시각의 각도를 복원하기 위한 것
        //
        // 예전에는 imageCallback 이 actual_pan_deg_ 를 그냥 읽었다.
        // 그런데 그건 50 Hz 콜백이 덮어쓰는 스칼라라 **처리 시각**
        // 의 값이고, 영상은 그보다 앞서 찍힌 것이다. 카메라가 도는
        // 중이면 그 사이 각도가 달라져 있으므로, 길이 L 경로의 먼
        // 끝이 L·Δθ 만큼 밀린다 (77cm 기준 3도 = 4.0cm, 15도 =
        // 19.9cm). pan 중에만 나타나는 튐의 유력 원인이었다.
        //
        // 스탬프를 달아 보관하고 영상 스탬프에서 선형보간한다.
        // 50 Hz 샘플 사이는 20 ms 이고 램프가 30 deg/s 이므로 그
        // 구간의 각도 변화는 0.6도, 선형보간 오차는 그보다 훨씬
        // 작다.
        //
        // 실기 주의: 서보 인코더가 없어 이 값들은 "실측"이 아니라
        // 명령의 에코다. 그래도 **명령 자체가 시간에 따라 변하므로**
        // (램프) 영상 시각의 명령각을 쓰는 것이 처리 시각의 명령각을
        // 쓰는 것보다 정확하다. 남는 오차는 서보 추종 지연이고,
        // 그건 인코더 없이는 관측 불가다.
        // ------------------------------------------------------------

        struct PanSample
        {
            rclcpp::Time stamp;

            double deg;
        };

        std::deque<PanSample> pan_history_;

        // 이력 보관 길이 [s]. 파이프라인 지연(수십 ms)의 10배 넘게
        // 잡아 두면 충분하고, 50 Hz x 1 s = 50 개라 비용도 없다.
        static constexpr double kPanHistorySec = 1.0;

        // 영상 스탬프에서의 pan 각도 [deg].
        // 이력이 비면 마지막 명령각(pan_cmd_deg_)으로 물러난다.
        // out_extrapolated 는 스탬프가 이력 범위 밖이라 끝값으로
        // 잘렸는지를 알려준다 (진단용).
        double panAngleAt(
            const rclcpp::Time & stamp,
            bool * out_extrapolated = nullptr) const;

        // 좌/우 흰선이 연속으로 안 잡힌 프레임 수
        int left_miss_streak_ = 0;

        int right_miss_streak_ = 0;

        // Holding 에서 연속으로 직선 판정이 난 프레임 수
        int pan_straight_streak_ = 0;

        // Searching 에서 연속으로 재검출 조건이 선 프레임 수.
        // Searching 에 들어갈 때 0 으로 되돌린다.
        int pan_found_streak_ = 0;

        // 직전 프레임의 꺾임각 [deg]. 진단 전용 (음수 = 판정 불가).
        // 복귀 판정에는 쓰이지 않는다 — status 의 pbend 로만 나간다.
        double last_bend_deg_ = -1.0;

        // 직전 프레임에 "정면에서도 보인다" 로 센 점 수. 진단 전용.
        int last_visible_at_zero_ = -1;

        // Holding 진입 시각. 기본 생성 rclcpp::Time 은 SYSTEM_TIME
        // 이라 node clock(ROS_TIME) 과 빼면 예외가 난다. 명시한다.
        rclcpp::Time pan_hold_start_{0, 0, RCL_ROS_TIME};


        std::vector<cv::Point2f> bev_src_points_;

        std::vector<cv::Point2f> bev_dst_points_;

        cv::Mat perspective_matrix_;


        // ================================================================
        // Camera Calibration
        // ================================================================

        cv::Mat camera_matrix_;

        cv::Mat distortion_coefficients_;

        bool camera_calibrated_;


        // undistort 재사용 맵.
        //
        // cv::undistort() 는 호출할 때마다 내부에서
        // initUndistortRectifyMap 을 다시 만든다 (480x360 float 맵
        // 두 장, 약 1.4 MB). intrinsic 은 CameraInfo 로 한 번 정해지면
        // 바뀌지 않으므로 맵도 한 번만 만들고 remap 만 돌린다.
        cv::Mat undistort_map1_;

        cv::Mat undistort_map2_;

        // 맵을 만든 영상 크기. 크기가 바뀌면 다시 만든다.
        cv::Size undistort_map_size_{0, 0};
        
        // path_ema_ctrl_ 의 실제 이동 속도 상한 [cm/s]. EMA 정상 블렌드는
        // 이미 완만하지만, 재잠금(relock) 순간은 통째로 덮어써 왔다 — 그게
        // "경로 튐"으로 보인다. 원인이 relock이든 spine 전환이든 무관하게,
        // 여기서 출력 자체의 이동 속도에 상한을 걸어 프레임 한 장 만에
        // 크게 뛰지 않게 한다 (publishPanRamped 와 같은 발상).
        //
        // 실측 전 초안값. 너무 낮으면 실제 경로 변화(급커브 진입 등)를
        // 따라잡는 데 지연이 생긴다.
        double path_max_step_cm_per_s_;

        rclcpp::Time path_ema_last_stamp_{0, 0, RCL_ROS_TIME};
        bool path_ema_time_init_ = false;
    };

    #endif  // KAU_LANE_DETECTION__KAU_LANE_DETECTION_NODE_HPP_