#include "SlamKernel.h"
#include <iostream>
#include <set>
#include <unordered_set>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include <core/System.h>
#include <utils/Settings.h>
#include <feature/ORBVocabulary.h>
#include "frame/KeyFrame.h"
#include "map/MapPoint.h"

#include <android/log.h>
#define DEBUG_INFO(...) ::__android_log_print(ANDROID_LOG_INFO, "[Android SLAM DEBUG]", __VA_ARGS__)

namespace android_slam
{

    SlamKernel::SlamKernel(int32_t img_width, int32_t img_height, std::string vocabulary_data, int64_t begin_time_stamp)
    : m_width(img_width)
    , m_height(img_height)
    , m_begin_time_stamp(begin_time_stamp)
    , m_last_time(std::chrono::steady_clock::now())
    {
        ORB_SLAM3::Settings::SettingDesc desc{};
        desc.sensor = ORB_SLAM3::System::eSensor::MONOCULAR;
        desc.cameraInfo.cameraType = ORB_SLAM3::Settings::CameraType::PinHole;
        //手机相机标定结果
        desc.cameraInfo.fx = 520.4708852845047f;
        desc.cameraInfo.fy = 523.7118931356812f;
        desc.cameraInfo.cx = 319.57018786503784f;
        desc.cameraInfo.cy = 238.20910561697508f;
        desc.distortion->k1 = -0.02640806687490354f;
        desc.distortion->k2 = 0.2033386379661157f;
        desc.distortion->p1 = 6.519121558220371e-06f;
        desc.distortion->p2 = -0.0009077232655942344f;
        /*
        desc.cameraInfo.fx = 458.654f;
        desc.cameraInfo.fy = 457.296f;
        desc.cameraInfo.cx = 367.215f;
        desc.cameraInfo.cy = 248.375f;
        desc.distortion->k1 = -0.28340811f;
        desc.distortion->k2 = 0.07395907f;
        desc.distortion->p1 = 0.00019359f;
        desc.distortion->p2 = 1.76187114e-05f;
         */
        desc.imageInfo.width = img_width;
        desc.imageInfo.height = img_height;
        desc.imageInfo.newWidth = 640;
        desc.imageInfo.newHeight = 480;
        desc.imageInfo.fps = 20;
        desc.imageInfo.bRGB = true;
        //手机imu标定结果
        desc.imuInfo.noiseGyro = 0.00037129946286789615f;
        desc.imuInfo.noiseAcc = 0.008926024139864043f;
        desc.imuInfo.gyroWalk = 4.310313880423259e-06f;
        desc.imuInfo.accWalk = 0.0002903411803301644f;
        /*
        desc.imuInfo.noiseGyro = 1.7e-4f;
        desc.imuInfo.noiseAcc = 2.0000e-3f;
        desc.imuInfo.gyroWalk = 1.9393e-05f;
        desc.imuInfo.accWalk = 3.0000e-03f;
         */
        desc.imuInfo.frequency = 200.0f;
        //手机联合标定结果
        desc.imuInfo.cvTbc = static_cast<cv::Mat>(cv::Mat_<float>(4, 4) << 0.0148655429818f, -0.999880929698f, 0.00414029679422f, -0.0216401454975f,
                0.999557249008f, 0.0149672133247f, 0.025715529948f, -0.064676986768f,
                -0.0257744366974f, 0.00375618835797f, 0.999660727178f, 0.00981073058949f,
                0.0f, 0.0f, 0.0f, 1.0f
        );
        //desc.imuInfo.cvTbc = static_cast<cv::Mat>(cv::Mat_<float>(4, 4) << +0.0148655429818f, -0.99988092969800f, +0.00414029679422f, -0.02164014549750f, +0.9995572490080f, +0.01496721332470f, +0.02571552994800f, -0.06467698676800f, -0.0257744366974f, +0.00375618835797f, +0.99966072717800f, +0.00981073058949f, 0.0f, 0.0f, 0.0f, 1.0f
        //);
        desc.imuInfo.bInsertKFsWhenLost = true;
        desc.orbInfo.nFeatures = 1000;
        desc.orbInfo.scaleFactor = 1.2f;
        desc.orbInfo.nLevels = 8;
        desc.orbInfo.initThFAST = 20;
        desc.orbInfo.minThFAST = 7;
        desc.viewerInfo.keyframeSize = 0.05f;
        desc.viewerInfo.keyframeLineWidth = 1.0f;
        desc.viewerInfo.graphLineWidth = 0.9f;
        desc.viewerInfo.pointSize = 2.0f;
        desc.viewerInfo.cameraSize = 0.08f;
        desc.viewerInfo.cameraLineWidth = 3.0f;
        desc.viewerInfo.viewPointX = 0.0f;
        desc.viewerInfo.viewPointY = -0.7f;
        desc.viewerInfo.viewPointZ = -3.5f;
        desc.viewerInfo.viewPointF = 500.0f;
        desc.viewerInfo.imageViewerScale = 1.0f;
        auto slam_settings = new ::ORB_SLAM3::Settings(desc);


        auto vocabulary = new ::ORB_SLAM3::ORBVocabulary();
        vocabulary->loadFromAndroidTextFile(std::move(vocabulary_data));


        m_orb_slam = std::make_unique<::ORB_SLAM3::System>(
        vocabulary, slam_settings, static_cast<const ORB_SLAM3::System::eSensor>(desc.sensor));

        pts_rel.push_back({0.0, 0.0, 0.0});
        pts_abs.push_back({0.0, 0.0, 0.0});
        //f304外方形走廊锚点，起点为f304外角
        pts_abs.push_back({9.0, 0.0, 0.0});
        pts_abs.push_back({38.4, 6.0, 0.0});
        //f304外方形走廊锚点，起点为f315外角，逆时针
        //pts_abs.push_back({9.0, 0.0, 0.0});
        //pts_abs.push_back({42.0, 6.0, 0.0});
        //f304锚点
        //pts_abs.push_back({0.0, 4.0, 0.0});
        //pts_abs.push_back({-3.0, 5.0, 0.0});

    }

    // unique_ptr needs to know how to delete the ptr, so the dtor should be impl with the definition of the ptr class.
    SlamKernel::~SlamKernel()
    {
        m_orb_slam->Shutdown();
    }

    void SlamKernel::reset()
    {
        m_orb_slam->Reset();
    }

    TrackingResult SlamKernel::handleData(const std::vector<Image>& images, const std::vector<ImuPoint>& imus, const float scale)
    {
        const Image& image = images[0];
        assert((image.time_stamp >= m_begin_time_stamp) && "Invalid time stamp.");

        cv::Mat cv_image(m_height, m_width, CV_8UC3);
        memcpy(cv_image.data, image.data.data(), sizeof(uint8_t) * image.data.size());


        double image_time_stamp = (double) (image.time_stamp - m_begin_time_stamp) * k_nano_sec_to_sec_radio;
        Sophus::SE3f pose = m_orb_slam->TrackMonocular(cv_image, image_time_stamp);


        Eigen::Matrix4f mat_pose = pose.matrix();
        TrackingResult res;
        {
            res.last_pose[+0] = mat_pose(0, 0);
            res.last_pose[+1] = mat_pose(1, 0);
            res.last_pose[+2] = mat_pose(2, 0);
            res.last_pose[+3] = mat_pose(3, 0);
            res.last_pose[+4] = mat_pose(0, 1);
            res.last_pose[+5] = mat_pose(1, 1);
            res.last_pose[+6] = mat_pose(2, 1);
            res.last_pose[+7] = mat_pose(3, 1);
            res.last_pose[+8] = mat_pose(0, 2);
            res.last_pose[+9] = mat_pose(1, 2);
            res.last_pose[10] = mat_pose(2, 2);
            res.last_pose[11] = mat_pose(3, 2);
            res.last_pose[12] = mat_pose(0, 3);
            res.last_pose[13] = mat_pose(1, 3);
            res.last_pose[14] = mat_pose(2, 3);
            res.last_pose[15] = mat_pose(3, 3);
        }

        ORB_SLAM3::Map* active_map = m_orb_slam->getAtlas().GetCurrentMap();
        if (active_map)
        {
            {
                std::vector<ORB_SLAM3::KeyFrame*> key_frames = active_map->GetAllKeyFrames();

                static auto key_frame_cmp = [](const ORB_SLAM3::KeyFrame* kf1, const ORB_SLAM3::KeyFrame* kf2)
                {
                    return kf1->mnFrameId < kf2->mnFrameId;
                };
                std::set<ORB_SLAM3::KeyFrame*, decltype(key_frame_cmp)> kf_set(key_frame_cmp);
                for (ORB_SLAM3::KeyFrame* kf: key_frames)
                {
                    if (!kf || kf->isBad())
                        continue;
                    kf_set.insert(kf);
                }

                for (ORB_SLAM3::KeyFrame* kf: kf_set)
                {
                    if (!kf || kf->isBad())
                        continue;

                    Eigen::Vector3f position = kf->GetPoseInverse().translation() * scale;
                    if(CalRt) {
                        Eigen::Vector3f position_trans = R * position + t;
                        res.trajectory.push_back({position_trans.x(), position_trans.y(), position_trans.z()});
                    }
                    else
                        res.trajectory.push_back({ position.x(), position.y(), position.z() });
                }

                Eigen::Vector3f last_position = pose.inverse().translation() * scale;
                if(CalRt) {
                    Eigen::Vector3f last_position_trans = R * last_position + t;
                    res.trajectory.push_back({last_position_trans.x(), last_position_trans.y(), last_position_trans.z()});
                }
                else
                    res.trajectory.push_back({ last_position.x(), last_position.y(), last_position.z() });
            }

            {
                std::vector<ORB_SLAM3::MapPoint*> local_mps = active_map->GetReferenceMapPoints();
                std::unordered_set<ORB_SLAM3::MapPoint*> local_mp_ust(local_mps.begin(), local_mps.end());

                for (ORB_SLAM3::MapPoint* mp: local_mps)
                {
                    if (!mp || mp->isBad())
                        continue;

                    Eigen::Vector3f position = mp->GetWorldPos();
                    res.map_points.push_back({ position.x(), position.y(), position.z() });
                }

                std::vector<ORB_SLAM3::MapPoint*> all_mps = active_map->GetAllMapPoints();
                for (ORB_SLAM3::MapPoint* mp: all_mps)
                {
                    if (!mp || mp->isBad() || local_mp_ust.find(mp) != local_mp_ust.end())
                        continue;

                    Eigen::Vector3f position = mp->GetWorldPos();
                    res.map_points.push_back({ position.x(), position.y(), position.z() });
                }
            }
        }

        {
            int status = m_orb_slam->getTrackingState();
            switch (status)
            {
                case -1:
                    res.tracking_status = "SYSTEM_NOT_READY";
                    break;
                case 0:
                    res.tracking_status = "NO_IMAGES_YET";
                    break;
                case 1:
                    res.tracking_status = "NOT_INITIALIZED";
                    break;
                case 2:
                    res.tracking_status = "OK";
                    break;
                case 3:
                    res.tracking_status = "RECENTLY_LOST";
                    break;
                case 4:
                    res.tracking_status = "LOST";
                    break;
                case 5:
                    res.tracking_status = "OK_KLT";
                    break;
                default:
                    break;
            }
        }

        {
            const std::chrono::steady_clock::time_point curr_time = m_last_time;
            m_last_time = std::chrono::steady_clock::now();

            res.processing_delta_time = std::chrono::duration<float>(m_last_time - curr_time).count();
        }

        return res;
    }

    void SlamKernel::CalculateRt() {
        if(pts_rel.size() == pts_abs.size()) {
            //1. 计算质心坐标
            cv::Point3f p1, p2;
            int N = pts_abs.size();
            for(int i = 0; i < N; i++) {
                p1 += pts_abs[i];
                p2 += pts_rel[i];
            }
            p1 = cv::Point3f(cv::Vec3f(p1) / N);
            p2 = cv::Point3f(cv::Vec3f(p2) / N);

            //2. 计算去质心坐标
            vector<cv::Point3f> q1(N);
            vector<cv::Point3f> q2(N);
            for(int i = 0; i < N; i++) {
                q1[i] = pts_abs[i] - p1;
                q2[i] = pts_rel[i] - p2;
            }

            //3. 计算矩阵W  q1 * q2^T
            Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
            for(int i = 0; i < N; i++)
                W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
            //cout << "W = \n" << W << endl;

            // 4. SVD 分解 W , 计算R
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d U = svd.matrixU();
            Eigen::Matrix3d V = svd.matrixV();
            if(U.determinant() * V.determinant() < 0) {
                for(int x = 0; x < 3; x++)
                    U(x, 2) *= -1;
            }
            R = (U*(V.transpose())).cast<float>();

            // 5. 由旋转矩阵R, 计算平移     t_ = p - Rp
            t = Eigen::Vector3f(p1.x, p1.y, p1.z) - R * Eigen::Vector3f(p2.x, p2.y, p2.z);

            setCalRt(true);
            pts_rel.clear();
            pts_rel.push_back({0.0, 0.0, 0.0});
        }
        else
            setCalRt(false);

        return;
    }

}