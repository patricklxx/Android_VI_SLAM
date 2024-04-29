#pragma once
#include <string>
#include <vector>

#include <android/asset_manager.h>

#include "SlamKernel.h"

#include <android/log.h>
#define DEBUG_INFO(...) ::__android_log_print(ANDROID_LOG_INFO, "[Android SLAM DEBUG]", __VA_ARGS__)

namespace android_slam
{
    class AssetManager
    {
    public:
        static bool getData(const std::string& path, int mode, std::vector<uint8_t>& data);

        static AAssetManager* get() { return s_asset_manager; }
        static void set(AAssetManager* manager) { s_asset_manager = manager; }

        static void writeFile();
        static std::string readFile(const std::string fileAdd);
        void saveTraj();
        void saveCurPos();
        void clearFile(const std::string fileAdd);
        void getTraj(TrackingResult tracking_res);

    private:
        static AAssetManager* s_asset_manager;
        TrackingResult result;
    };

}