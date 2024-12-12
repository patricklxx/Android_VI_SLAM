#include  "utils/Communicator.h"

#include <android/log.h>
#define DEBUG_INFO(...) ::__android_log_print(ANDROID_LOG_INFO, "[Android SLAM DEBUG]", __VA_ARGS__)

namespace android_slam
{
    Communicator::Communicator()
    {
        DEBUG_INFO("[Android Slam App Info] Communicator is built.");
    }

    void Communicator::Run(float x, float y, float z, std::string ip, std::string port)
    {
        int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd == -1)
        {
            //std::cout << "socket 创建失败：" << std::endl;
            DEBUG_INFO("[Android Slam App Info] Socket create failed.");
            return;
        }
        DEBUG_INFO("[Android Slam App Info] Socket create succesfully.");
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        //addr.sin_port = htons(8051);
        //addr.sin_addr.s_addr = inet_addr("10.134.114.59");
        addr.sin_port = htons(std::stoi(port));
        addr.sin_addr.s_addr = inet_addr(ip.c_str());

        int res = connect(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
        if (res == -1)
        {
            DEBUG_INFO("[Android Slam App Info] Socket connect failed.");
            return;
        }
        DEBUG_INFO("[Android Slam App Info] Socket connect successfully.");

        float commu_data[4] = {x, y, z, 1.0};
        //std::cout << commu_data[0] << " " << commu_data[1] << " " << commu_data[2] << " " << commu_data[3] << std::endl;
        DEBUG_INFO("[Android Slam App Info] Socket send data: x: %f, y: %f, z: %f.", x, y, z);
        send(socket_fd, (char *)commu_data, sizeof(commu_data), 0);
        close(socket_fd);
        //usleep(100000);
        return;
    }

    void Communicator::Run(TrackingResult track_res, std::string ip, std::string port)
    {
        int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd == -1)
        {
            //std::cout << "socket 创建失败：" << std::endl;
            DEBUG_INFO("[Android Slam App Info] Socket create failed.");
            return;
        }
        DEBUG_INFO("[Android Slam App Info] Socket create succesfully.");
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        //addr.sin_port = htons(9090);
        //addr.sin_addr.s_addr = inet_addr("192.168.43.172");
        addr.sin_port = htons(std::stoi(port));
        addr.sin_addr.s_addr = inet_addr(ip.c_str());

        int res = connect(socket_fd, (struct sockaddr *)&addr, sizeof(addr));

        if (res == -1)
        {
            DEBUG_INFO("[Android Slam App Info] Socket connect failed.");
            return;
        }
        DEBUG_INFO("[Android Slam App Info] Socket connect successfully.");

        uint32_t size = track_res.trajectory.size() * 3;
        send(socket_fd, (char *)&size, sizeof(uint32_t), 0);
        DEBUG_INFO("[Android Slam App Info] sizeof(uint32_t) = %d", sizeof(uint32_t));

        float commu_data[size];
        int count = 0;
        for(auto &data : track_res.trajectory)
        {
            commu_data[count++] = data.x + start_pos[0];
            //commu_data[count++] = data.y + start_pos[1];
            //commu_data[count++] = data.z + start_pos[2];
            commu_data[count++] = data.z + start_pos[1];
            commu_data[count++] = data.y + start_pos[2];
        }
        send(socket_fd, (char *)commu_data, sizeof(commu_data), 0);
        DEBUG_INFO("[Android Slam App Info] sizeof(commu_data) = %d", sizeof(commu_data));
        close(socket_fd);
        //usleep(100000);
        return;
    }

    void Communicator::getStartPos(std::string  start_str) {
        //识别初始位置
        //DEBUG_INFO("[Android Slam App Info] start_pos = (%f, %f, %f)", start_pos[0], start_pos[1], start_pos[2]);
        std::string pos_str;
        int num = 0;
        for(int i = 0; i < start_str.size(); ++i) {
            if(start_str[i] != ' ')
                pos_str += start_str[i];
            else {
                start_pos[num++] = std::stof(pos_str);
                pos_str.clear();
            }
        }
        DEBUG_INFO("[Android Slam App Info] start_pos = (%f, %f, %f)", start_pos[0], start_pos[1], start_pos[2]);
    }

}

