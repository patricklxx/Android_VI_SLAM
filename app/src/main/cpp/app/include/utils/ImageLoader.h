#pragma once
#include <memory>

#include "render/ImageTexture.h"
#include "render/ImagePool.h"

namespace android_slam
{

    class ImageLoader
    {
    public:
        static std::unique_ptr<ImageTexture> createImage(const char* file);
        static std::vector<Image> loadDatasetImage(const char* file);
        static std::unique_ptr<ImageTexture> createImagenew(const char* file);
        static std::unique_ptr<ImageTexture> createImageDataset(const char* file);
        static std::vector<ImuPoint> loadDatasetImu(const char* file);
        static std::vector<ImuPoint> getImuDataset(std::vector<ImuPoint> imu_data, int64_t current_time, int64_t last_time);
    };

}