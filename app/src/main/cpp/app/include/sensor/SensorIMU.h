#pragma once
#include <vector>
#include <queue>
#include <tuple>

#include <android/sensor.h>

#include <SlamKernel.h>

namespace android_slam
{

    class SensorIMU
    {
    private:
        static constexpr uint32_t k_sensor_refresh_rate_hz = 200;
        static constexpr int32_t k_sensor_refresh_delta_time_us = int32_t(1000000 / k_sensor_refresh_rate_hz);
        static constexpr const char* k_package_name = "cntlab.f304.androidslam";

    public:
        SensorIMU();
        SensorIMU(const SensorIMU&) = delete;
        SensorIMU& operator=(const SensorIMU&) = delete;
        ~SensorIMU() noexcept;

        std::vector<ImuPoint> getImuData() const;

    private:
        ASensorManager* m_sensor_manager = nullptr;
        ALooper* m_looper = nullptr;

        const ASensor* m_accelerometer = nullptr;
        ASensorEventQueue* m_accelerometer_event_queue = nullptr;

        const ASensor* m_gyroscope = nullptr;
        ASensorEventQueue* m_gyroscope_event_queue = nullptr;

        mutable std::queue<std::tuple<float, float, float>> m_acce_data_que;
        mutable std::queue<std::tuple<float, float, float>> m_gyro_data_que;
    };

}