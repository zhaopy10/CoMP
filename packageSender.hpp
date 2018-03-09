#ifndef PACKAGESENDER
#define PACKAGESENDER

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
#include <unistd.h>
#include <chrono>
#include <pthread.h>

typedef unsigned short ushort;

class PackageSender
{
public:
    static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    // int for: frame_id, subframe_id, cell_id, ant_id
    // unsigned int for: I/Q samples
    static const int buffer_length = sizeof(int) * 4 + sizeof(ushort) * OFDM_FRAME_LEN * 2;
    static const int data_offset = sizeof(int) * 4;
    static const int subframe_num_perframe = 40;

public:
    PackageSender(int in_thread_num);
    ~PackageSender();

    void genData();
    void loopSend();
    
private:
    struct sockaddr_in servaddr_;    /* server address */
    struct sockaddr_in cliaddr_;    /* server address */
    int* socket_;
    std::vector<std::vector<char> > buffer_;
    int frame_id;
    int subframe_id;

    float** IQ_data;
    ushort** IQ_data_coded;

    int thread_num;

};


#endif