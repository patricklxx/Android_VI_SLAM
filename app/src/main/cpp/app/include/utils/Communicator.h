#pragma once
#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <SlamKernel.h>

namespace android_slam
{
    class Communicator
    {
    public:
        Communicator();
        void Run(float x, float y, float z,std::string ip, std::string port);
        void Run(TrackingResult track_res, std::string ip, std::string port);
        void Run(TrackingResult track_res, std::string ip, std::string port, std::string start_str);
        void getStartPos(std::string start_str);

    private:
        float start_pos[3] = {0.0, 0.0, 0.0};
        //std::vector<float> start_pos;
    };
}