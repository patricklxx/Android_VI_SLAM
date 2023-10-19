#include  "utils/Communicator.h"

#include <android/log.h>
#define DEBUG_INFO(...) ::__android_log_print(ANDROID_LOG_INFO, "[Android SLAM DEBUG]", __VA_ARGS__)

namespace android_slam
{
    Communicator::Communicator()
    {
        DEBUG_INFO("[Android Slam App Info] Communicator Start.");
    }

    void Communicator::Run(float x, float y, float z)
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
        addr.sin_port = htons(8888);
        addr.sin_addr.s_addr = inet_addr("10.134.115.5");

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

}

