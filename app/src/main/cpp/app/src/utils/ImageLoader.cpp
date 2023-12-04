#include "utils/ImageLoader.h"
#include <sstream>

#include "utils/AssetManager.h"
#include "utils/Log.h"

#define STB_IMAGE_IMPLEMENTATION
#include "utils/stb_image.h"

namespace android_slam
{
namespace image_loader_utils
{

    typedef union
    {
        int32_t i32;
        uint8_t ui8[4];
    } bit32_t;

}

    std::unique_ptr<ImageTexture> ImageLoader::createImage(const char* file)
    {
        AAsset* asset = AAssetManager_open(AssetManager::get(), file, AASSET_MODE_BUFFER);
        assert(asset && "[Android Slam Shader Info] Failed to open shader file.");

        size_t size = AAsset_getLength(asset);
        auto data = (const uint8_t*)AAsset_getBuffer(asset);

        image_loader_utils::bit32_t width;
        image_loader_utils::bit32_t height;
        memcpy(width.ui8, data, sizeof(image_loader_utils::bit32_t));
        memcpy(height.ui8, data + 4, sizeof(image_loader_utils::bit32_t));

        assert(size == (width.i32 * height.i32 * 3 + 8));


        std::vector<uint8_t> image_data(width.i32 * height.i32 * 3);
        memcpy(image_data.data(), data + 8, sizeof(uint8_t) * image_data.size());

        AAsset_close(asset);

        return std::make_unique<ImageTexture>(width.i32, height.i32, image_data);
    }

    std::vector<Image> ImageLoader::loadDatasetImage(const char* file)
    {
        AAssetDir* assetDir = AAssetManager_openDir(AssetManager::get(), file);
        std::vector<Image> DatasetImg;
        if (NULL != assetDir)
        {
            const char* cOpen = AAssetDir_getNextFileName(assetDir);
            while (NULL != cOpen)
            {
                // 转成全路径
                std::string strOpen = cOpen;
                std::string strFullOpen = file;
                strFullOpen += ("/" + strOpen);
                //DEBUG_INFO("[ImageLoader]: FullOpen: %s",strFullOpen.c_str());
                if(strOpen[0] != 'a')
                {
                    AAsset* asset = AAssetManager_open(AssetManager::get(), strFullOpen.c_str(), AASSET_MODE_BUFFER);
                    size_t size = AAsset_getLength(asset);
                    auto data = (const uint8_t*)AAsset_getBuffer(asset);

                    int32_t type = 0;
                    int32_t width = 0;
                    int32_t height = 0;
                    stbi_set_flip_vertically_on_load(true);
                    uint8_t* imgData = stbi_load_from_memory(data, size, &width, &height, &type, 0);

                    int32_t imgSize = width * height * 3;
                    std::vector<uint8_t> image_data(imgSize);
                    if(imgSize > 0 && imgData != nullptr)
                        memcpy(image_data.data(), imgData, sizeof(uint8_t) * image_data.size());

                    //时间戳
                    int64_t time_stamp;
                    /*
                    std::string time_str = strOpen.erase(10,1);
                    time_str = time_str.substr(5,9);
                    time_stamp = std::stol(time_str) * 100000;
                    //DEBUG_INFO("[ImageLoader]: camera_time_stamp: %ld",time_stamp);
                    DatasetImg.push_back(Image{ std::move(image_data), time_stamp });
                     */
                    time_stamp = std::stol(strOpen);
                    DEBUG_INFO("[ImageLoader]: camera_time_stamp: %ld",time_stamp);
                    DatasetImg.push_back(Image{ std::move(image_data), time_stamp });

                    AAsset_close(asset);
                }
                else break;
                // todo.
                cOpen = AAssetDir_getNextFileName(assetDir);

            }
            AAssetDir_close(assetDir);
            //return DatasetImg;
        }
        return DatasetImg;
    }

    std::unique_ptr<ImageTexture> ImageLoader::createImagenew(const char* file)
    {
        AAsset* asset = AAssetManager_open(AssetManager::get(), file, AASSET_MODE_BUFFER);
        assert(asset && "[Android Slam Shader Info] Failed to open shader file.");

        size_t size = AAsset_getLength(asset);
        auto data = (const uint8_t*)AAsset_getBuffer(asset);

        int32_t type = 0;
        int32_t width = 0;
        int32_t height = 0;
        stbi_set_flip_vertically_on_load(true);
        uint8_t* imgData = stbi_load_from_memory(data, size, &width, &height, &type, 0);

        int32_t imgSize = width * height * 3;
        std::vector<uint8_t> image_data(imgSize);
        if(imgSize > 0 && imgData != nullptr)
            memcpy(image_data.data(), imgData, sizeof(uint8_t) * image_data.size());
        AAsset_close(asset);

        return std::make_unique<ImageTexture>(width, height, image_data);
    }

    std::unique_ptr<ImageTexture> ImageLoader::createImageDataset(const char* file)
    {
        AAssetDir* assetDir = AAssetManager_openDir(AssetManager::get(), file);
        std::unique_ptr<ImageTexture> img_d;
        if (NULL != assetDir)
        {
            const char* cOpen = AAssetDir_getNextFileName(assetDir);

            while (NULL != cOpen)
            {
                std::string strOpen = cOpen;
                std::string strFullOpen = file;
                strFullOpen += ("/" + strOpen);
                //DEBUG_INFO("[ImageLoader]: FullOpen: %s",strFullOpen.c_str());
                if(strOpen[0] != 'a')
                {
                    img_d = createImagenew(strFullOpen.c_str());
                    cOpen = AAssetDir_getNextFileName(assetDir);
                }
                else break;
            }

            AAssetDir_close(assetDir);
        }
        return img_d;
    }
    std::vector<ImuPoint> ImageLoader::loadDatasetImu(const char *file)
    {
        AAsset* asset = AAssetManager_open(AssetManager::get(), file, AASSET_MODE_BUFFER);
        assert(asset && "[Android Slam Shader Info] Failed to open shader file.");

        size_t size = AAsset_getLength(asset);
        auto data = (const uint8_t*)AAsset_getBuffer(asset);

        std::vector<char> data_char(data, data + size);
        std::vector<std::string> data_str;
        std::vector<float> dataImu;
        std::vector<int64_t> dataTime;
        std::string s;

        for(int i = 0; i < size; ++i)
        {
            //DEBUG_INFO("[ImageLoader]: imu_data: %c",data_char[i]);
            if(data_char[i] != ' ' && data_char[i] != '\n')
            {
                s.push_back(data_char[i]);
                //DEBUG_INFO("[ImageLoader]: data_str: %s",s.c_str());
            }
            else
            {
                //DEBUG_INFO("[ImageLoader]: s: %s",s.c_str());
                if(s.find('e') != std::string::npos)//判断是否为时间戳
                {
                    std::string time_str = s.erase(1,1);
                    time_str = time_str.substr(5,9);
                    int64_t time_stamp = std::stol(time_str) * 100000;
                    if((time_stamp - dataTime[dataTime.size() - 1]) < 1e+9 && (time_stamp - dataTime[dataTime.size() - 1]) > 0)
                        dataTime.push_back(time_stamp);
                    //DEBUG_INFO("[ImageLoader]: dataTime: %ld",dataTime[dataTime.size()-1]);
                }
                else
                {
                    dataImu.push_back(std::stof(s));
                    //DEBUG_INFO("[ImageLoader]: imu_data: %f",dataImu[dataImu.size()-1]);
                }
                s.clear();
            }
            if(i == size - 1)
            {
                dataImu.push_back(std::stof(s));
                //DEBUG_INFO("[ImageLoader]: imu_data: %f",dataImu[data_size]);
            }
        }
        AAsset_close(asset);

        std::vector<ImuPoint> imu_data;
        int time_num = 0;
        for(int i = 0; i < dataImu.size(); i+=6)
        {
            if((dataTime[time_num] < dataTime[time_num + 1]))//&& ((time_num + 1) < dataTime.size())
            {
                imu_data.push_back({dataImu[i + 3], dataImu[i + 4], dataImu[i + 5],
                                    dataImu[i], dataImu[i + 1], dataImu[i + 2],
                                    int64_t(dataTime[time_num])});
            }
            time_num++;
        }
        for(int i = 0; i < imu_data.size(); ++i)
        {
            //DEBUG_INFO("[ImageLoader]: az = %f",imu_data[i].az);
            DEBUG_INFO("[ImageLoader]: imu_time_stamp = %ld",imu_data[i].time_stamp);
        }
        return imu_data;
    }

    std::vector<ImuPoint> ImageLoader::getImuDataset(std::vector<ImuPoint> imu_data, int64_t current_time, int64_t last_time)
    {
        std::vector<ImuPoint> currentImu;

        for(int i = 0; i < imu_data.size(); ++i)
        {
            if(imu_data[i].time_stamp > last_time && imu_data[i].time_stamp < current_time)
                currentImu.push_back(imu_data[i]);
            else if(imu_data[i].time_stamp > current_time)
                break;
        }
        return currentImu;
    }

}