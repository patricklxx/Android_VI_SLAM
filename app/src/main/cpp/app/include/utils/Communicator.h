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
        void Run(float x, float y, float z);
        void Run(TrackingResult track_res);
    };



}