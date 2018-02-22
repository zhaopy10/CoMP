#ifndef PACKAGERECEIVER
#define PACKAGERECEIVER

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <stdlib.h>
#include "Symbols.hpp"
#include <vector>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <pthread.h>
#include <cassert>
#include <unistd.h>
#include <chrono>

class PackageReceiver
{
public:

    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // float for: I/Q samples
    static const int package_length = sizeof(int) * 4 + sizeof(float) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    

public:
    PackageReceiver();
    PackageReceiver(int* in_pipe);
    ~PackageReceiver();

    pthread_t startRecv(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int core_id = 1);
    static void* loopRecv(void *context);
 
private:
    struct sockaddr_in servaddr_;    /* server address */
    int socket_;

    char* buffer_;
    int* buffer_status_;
    int buffer_length_;
    int buffer_frame_num_;

    int* pipe_;  // write at port 1
    int core_id_;
};


#endif