#include "packageReceiver.hpp"


PackageReceiver::PackageReceiver()
{
    if ((socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("cannot create socket\n");
        exit(0);
    }
    int rcvbufsize = sizeof(float) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * subframe_num_perframe;
    setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &rcvbufsize, sizeof(rcvbufsize));  

    /*Configure settings in address struct*/
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(7891);
    servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  

        /*Bind socket with address struct*/
    if(bind(socket_, (struct sockaddr *) &servaddr_, sizeof(servaddr_)) != 0)
        perror("socket bind failed");

    /* initialize random seed: */
    srand (time(NULL));

}

PackageReceiver::PackageReceiver(int* in_pipe):
PackageReceiver()
{
    pipe_ = in_pipe;
}

PackageReceiver::~PackageReceiver()
{
}

pthread_t PackageReceiver::startRecv(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == package_length * buffer_frame_num_); // should be integre
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    //printf("start Recv thread\n");
    // new thread
    pthread_t recv_thread_;
    if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)this) != 0)
    {
        perror("socket recv thread create failed");
        exit(0);
    }
    return recv_thread_;
}

void* PackageReceiver::loopRecv(void *context)
{
    printf("package receiver thread start\n");
    PackageReceiver* obj_ptr = (PackageReceiver *)context;
    char* buffer = obj_ptr->buffer_;
    int* buffer_status = obj_ptr->buffer_status_;
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;
    int* pipe = obj_ptr->pipe_;

    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
    // loop recv
    socklen_t addrlen = sizeof(obj_ptr->servaddr_);
    int offset = 0;
    int package_num = 0;
    clock_t begin = clock();
    while(true)
    {
        if(cur_ptr_buffer_status[0] == 1)
        {
            perror("buffer full");
            exit(0);
        }

        int recvlen = -1;
        int ant_id, frame_id, subframe_id, cell_id;
        if ((recvlen = recvfrom(obj_ptr->socket_, (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_, &addrlen)) < 0)
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

        // write pipe
        write(pipe[1], (char *)&offset, sizeof(int));

        package_num++;
        if(package_num == 1e5)
        {
            clock_t end = clock();
            double byte_len = sizeof(float) * OFDM_FRAME_LEN * 2 * 1e5;
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            printf("receive %f bytes in %f secs, throughput %f MB/s\n", byte_len, elapsed_secs, byte_len / elapsed_secs / 1024 / 1024);
            begin = clock();
            package_num = 0;
        }
    }

}
