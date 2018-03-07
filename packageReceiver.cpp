#include "packageReceiver.hpp"
#include "cpu_attach.hpp"
#define ENABLE_CPU_ATTACH
PackageReceiver::PackageReceiver(int N_THREAD)
{
    socket_ = new int[N_THREAD];
    /*Configure settings in address struct*/
    
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(7891);
    servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  
    
    for(int i = 0; i < N_THREAD; i++)
    {
        if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("cannot create socket %d\n", i);
            exit(0);
        }

        int optval = 1;
        setsockopt(socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

        if(bind(socket_[i], (struct sockaddr *) &servaddr_, sizeof(servaddr_)) != 0)
        {
            printf("socket bind failed %d\n", i);
            exit(0);
        }

    }
    

    thread_num_ = N_THREAD;
    /* initialize random seed: */
    srand (time(NULL));
    context = new PackageReceiverContext[thread_num_];

}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] context;
}

std::vector<pthread_t> PackageReceiver::startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == package_length * buffer_frame_num_); // should be integre
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    //printf("start Recv thread\n");
    // new thread
    
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++)
    {
        pthread_t recv_thread_;
        
        context[i].ptr = this;
        context[i].tid = i;

        if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)(&context[i])) != 0)
        {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
    
    return created_threads;
}

void* PackageReceiver::loopRecv(void *in_context)
{
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(tid) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, tid + 1);
        exit(0);
    }
#endif

    char* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
    // loop recv
    socklen_t addrlen = sizeof(obj_ptr->servaddr_);
    int offset = 0;
    int package_num = 0;
    auto begin = std::chrono::system_clock::now();

    while(true)
    {
        /*
        if(cur_ptr_buffer_status[0] == 1)
        {
            perror("buffer full");
            exit(0);
        }
        */
        int recvlen = -1;
        int ant_id, frame_id, subframe_id, cell_id;
        if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_, &addrlen)) < 0)
        {
            perror("recv failed");
            exit(0);
        }
        frame_id = *((int *)cur_ptr_buffer);
        subframe_id = *((int *)cur_ptr_buffer + 1);
        cell_id = *((int *)cur_ptr_buffer + 2);
        ant_id = *((int *)cur_ptr_buffer + 3);
        //printf("receive frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", frame_id, subframe_id, cell_id, ant_id);
        
        offset = cur_ptr_buffer_status - buffer_status;
        // move ptr
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to false
        cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + 1) % buffer_frame_num;
        cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + package_length) % buffer_length;

        package_num++;
        if(package_num == 1e5)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(float) * OFDM_FRAME_LEN * 2 * 1e5;
            std::chrono::duration<double> diff = end - begin;
            printf("thread %d receive %f bytes in %f secs, throughput %f MB/s\n", tid, byte_len, diff.count(), byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
            package_num = 0;
        }
    }

}
