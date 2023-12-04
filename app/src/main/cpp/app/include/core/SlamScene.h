#pragma once
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>

#include <SlamKernel.h>

#include "core/Scene.h"

#include "sensor/SensorCamera.h"
#include "sensor/SensorIMU.h"

#include "render/ImagePool.h"
#include "render/SlamRenderer.h"

#include "utils/Timer.h"
#include "utils/Communicator.h"


namespace android_slam
{

    class SlamScene : public Scene
    {
    private:
        static constexpr int32_t k_sensor_camera_width = 640;
        static constexpr int32_t k_sensor_camera_height = 480;

        static constexpr float   k_fps_camera_fov = 45.0f;
        static constexpr float   k_fps_camera_z_min = 0.1f;
        static constexpr float   k_fps_camera_z_max = 1000.0f;

    public:
        explicit SlamScene(App& app, const char* name) noexcept
            : Scene(app, name)
        {}
        SlamScene(const SlamScene&) = delete;
        SlamScene& operator=(const SlamScene&) = delete;

        void init() override;
        void exit() override;

        void update(float dt) override;
        void drawGui(float dt) override;


    private:
        std::unique_ptr<ImagePool>    m_image_pool;
        std::unique_ptr<SensorIMU>    m_imu_pool;
        std::unique_ptr<SlamRenderer> m_slam_renderer;
        std::unique_ptr<SlamKernel>   m_slam_kernel;

        std::unique_ptr<std::thread> m_slam_thread;
        std::vector<Image>           m_images;
        std::vector<ImuPoint>        m_imu_points;

        TrackingResult               m_tracking_result;
        std::mutex                   m_image_mutex;
        std::mutex                   m_tracking_res_mutex;
        std::atomic_bool             m_slam_has_new_data = false;
        std::atomic_bool             m_is_running_slam = true;

        bool m_need_update_image = true;

        bool m_open_comm = false;
        std::unique_ptr<std::thread> m_comm_thread;
        std::mutex                   m_comm_mutex;
        std::atomic_bool             m_comm_has_new_data = false;
        TrackingResult               m_comm_result;
        Communicator comm;

        bool m_from_datasets = 0; //是否从数据集中获取离线数据
        int m_datasets_num = 1; //数据集传入的第几张图片
        std::vector<Image> DatasetImage;
        std::vector<ImuPoint> DatasetImu;
    };

}