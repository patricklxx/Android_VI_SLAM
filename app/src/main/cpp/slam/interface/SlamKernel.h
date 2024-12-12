#pragma once
#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <array>

#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace ORB_SLAM3
{
    class System;
}

namespace android_slam
{

    struct Image
    {
        std::vector<uint8_t> data;
        int64_t time_stamp;
    };

    struct ImuPoint
    {
        float ax;
        float ay;
        float az;
        float wx;
        float wy;
        float wz;
        int64_t time_stamp;
    };

    struct TrackingResult
    {
        struct Pos
        {
            float x;
            float y;
            float z;
        };

        std::array<float, 16> last_pose;
        std::vector<Pos> trajectory;
        std::vector<Pos> map_points;
        std::string tracking_status;

        float processing_delta_time;
    };

    class SlamKernel
    {
    private:
        static constexpr int64_t k_nano_second_in_one_second = 1000000000;
        static constexpr double k_nano_sec_to_sec_radio = 1.0 / (double)(k_nano_second_in_one_second);

    public:
        SlamKernel(int32_t img_width, int32_t img_height, std::string vocabulary_data, int64_t begin_time_stamp);
        SlamKernel(const SlamKernel&) = delete;
        SlamKernel& operator=(const SlamKernel&) = delete;
        ~SlamKernel();

        TrackingResult handleData(const std::vector<Image>& images, const std::vector<ImuPoint>& imus, const float scale);

        void reset();

        void setPtsRel(float x, float y, float z) { pts_rel.push_back({x, y, z}); }
        cv::Point3f getFirstPtsRel() { return pts_rel[0]; }
        int getPtsRelSize() { return pts_rel.size(); }
        void setCalRt(bool flag) { CalRt = flag; }
        void CalculateRt();

    private:
        int32_t m_width;
        int32_t m_height;
        const int64_t m_begin_time_stamp;

        std::unique_ptr<::ORB_SLAM3::System> m_orb_slam;

        std::chrono::steady_clock::time_point m_last_time;

        std::vector<cv::Point3f> pts_abs;
        std::vector<cv::Point3f> pts_rel;
        Eigen::Matrix3f R;
        Eigen::Vector3f t;
        bool CalRt = false;

    };

}