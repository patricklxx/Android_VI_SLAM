#include "utils/AssetManager.h"
#include <cassert>

namespace android_slam
{

    AAssetManager* AssetManager::s_asset_manager = nullptr;

    bool AssetManager::getData(const std::string& path, int mode, std::vector<uint8_t>& data)
    {
        AAsset* asset = AAssetManager_open(s_asset_manager, path.c_str(), mode);

        if(!asset) return false;

        size_t size = AAsset_getLength(asset);
        data.resize(size);

        AAsset_read(asset, data.data(), size);

        AAsset_close(asset);
        return true;
    }


    std::string AssetManager::readFile(const std::string fileAdd)
    {
        //const char* path = "/storage/emulated/0/Documents/IPAdd.txt";//共享存储路径
        //const char* path = "/storage/emulated/0/Android/data/cntlab.f304.androidslam/files/test.txt";//应用专属存储空间
        const char* path = fileAdd.c_str();
        std::string content;
        FILE *file;
        file = fopen(path, "r");
        if (file != NULL) {
            //读文件成功"
            DEBUG_INFO("[AssetManager::readFile] Open File successfully");
            char buffer[1024]={0};
            while(fread(buffer, sizeof(char),1,file)!=0) {
                content += buffer;
                //DEBUG_INFO("[AssetManager::readFile] ipadd:%s",buffer);
            }
            fclose(file);
        }
        else {
            //"读文件失败"
            int errNum = errno;
            DEBUG_INFO("[AssetManager::readFile] open fail errno = %d reason = %s \n", errNum,  strerror(errNum));
        }
        return content;
    }

    void AssetManager::writeFile()
    {
        const char* path = "/storage/emulated/0/Pictures/test.txt";//共享存储路径
        //const char* path = "/storage/emulated/0/Android/data/cntlab.f304.androidslam/files/test.txt";//应用专属存储空间
        FILE *file;
        file = fopen(path, "wb");
        if (file != NULL) {
            //打开文件成功"
            DEBUG_INFO("[AssetManager::readFile] Open file successfully");
            char buffer[1024]={0};
            buffer[0] = 'h';
            buffer[1] = 'e';
            buffer[2] = 'l';
            buffer[3] = 'l';
            buffer[4] = 'o';
            size_t count = fwrite(buffer, sizeof(char),5,file);
            DEBUG_INFO("[AssetManager::readFile] fread : buffer = %s , write count = %u\n", buffer, count);

            fclose(file);
        } else {
            //"读文件失败"
            int errNum = errno;
            DEBUG_INFO("[AssetManager::readFile] Open file failed");
            DEBUG_INFO("[AssetManager::readFile] open fail errno = %d reason = %s \n", errNum,  strerror(errNum));
        }
        return;

    }
    void AssetManager::saveTraj()
    {
        const char* path = "/storage/emulated/0/Documents/trajectory.txt";//共享存储路径
        FILE *file;
        file = fopen(path, "wb");
        if (file != NULL) {
            //打开文件成功"
            DEBUG_INFO("[AssetManager::savetraj] Open file successfully");
            for(int i = 0; i < result.trajectory.size(); ++i)
                //fprintf(file, "%f %f %f\n", result.trajectory[i].x, result.trajectory[i].y, result.trajectory[i].z);
                fprintf(file, "%f %f %f\n", result.trajectory[i].x, result.trajectory[i].z, result.trajectory[i].y);
            DEBUG_INFO("[AssetManager::savetraj] finish saving ");
            fclose(file);
        } else {
            //"打开文件失败"
            int errNum = errno;
            DEBUG_INFO("[AssetManager::savetraj] open fail errno = %d reason = %s \n", errNum,  strerror(errNum));
        }
        return;
    }

    void AssetManager::saveCurPos() {
        const char* path = "/storage/emulated/0/Documents/MarkPos.txt";//共享存储路径
        FILE *file;
        file = fopen(path, "a");
        if (file != NULL) {
            //打开文件成功"
            DEBUG_INFO("[AssetManager::saveCurPos] Open file successfully");
            //fprintf(file, "%f %f %f\n", result.trajectory.back().x, result.trajectory.back().y, result.trajectory.back().z);
            fprintf(file, "%f %f %f\n", result.trajectory.back().x, result.trajectory.back().z, result.trajectory.back().y);
            DEBUG_INFO("[AssetManager::saveCurPos] finish saving ");
            fclose(file);
        } else {
            //"打开文件失败"
            int errNum = errno;
            DEBUG_INFO("[AssetManager::saveCurPos] open fail errno = %d reason = %s \n", errNum,  strerror(errNum));
        }
        return;
    }

    void AssetManager::clearFile(const std::string fileAdd) {
        const char* path = fileAdd.c_str();
        FILE *file;
        file = fopen(path, "wb");
        if (file != NULL) {
            //读文件成功"
            DEBUG_INFO("[AssetManager::clearFile] clear File successfully");
            fclose(file);
        }
        else {
            //"读文件失败"
            int errNum = errno;
            DEBUG_INFO("[AssetManager::readFile] open fail errno = %d reason = %s \n", errNum,  strerror(errNum));
        }
        return;
    }

    void AssetManager::getTraj(TrackingResult tracking_res)
    {
        result = tracking_res;
        return;
    }

}