/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"
//#include "utils.hpp"
static void loadData(char* filename, std::vector<std::complex<int16_t>> &data, int samples)
{
        printf("entering loadData ... \n");
	FILE* fp = fopen(filename,"r");
	data.resize(samples);
	float real, imag;
	for(int i = 0; i < samples; i++)
	{
		fscanf(fp, "%f %f", &real, &imag);
                data[i] = std::complex<int16_t>(int16_t(real*32768), int16_t(imag*32768));
	}

	fclose(fp);
}

static void printVector(std::vector<std::complex<int16_t>> &data)
{
    for(int i = 0; i < data.size(); i++)
    {
        std::cout << real(data.at(i)) << " " << imag(data.at(i)) << std::endl;
    }
}


PackageReceiver::PackageReceiver(int N_THREAD)
{
#ifdef USE_SOCKET
    socket_ = new int[N_THREAD];
    /*Configure settings in address struct*/
    // address of sender 
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
        // use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance
        int optval = 1;
        setsockopt(socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

        if(bind(socket_[i], (struct sockaddr *) &servaddr_, sizeof(servaddr_)) != 0)
        {
            printf("socket bind failed %d\n", i);
            exit(0);
        }

    }
#else

    std::vector<std::string> bs_radio_ids;
    // FIXME: configuration should be read from file
    bs_radio_ids.push_back("0378");
    int maxFrame = 1 << 31;
    int symnum = 20;
    int symlen = 256;
    std::vector<int> tdd_sched(symnum);
    std::fill(tdd_sched.begin(), tdd_sched.end(), 0);
    tdd_sched[0] = 1;
    tdd_sched[2] = 2;
    tdd_sched[3] = 2;

    radioconfig_ = new RadioConfig(bs_radio_ids, 2, 5e6, 915e6, 20, 50, symlen, symnum, maxFrame, tdd_sched);

    // FIXME: multithread me
    std::vector<std::complex<int16_t>> beacon;
    loadData("beacon.txt", beacon, symlen);
    printVector(beacon);
    std::vector<void * > buffer_beacon(2);
    buffer_beacon[0] = beacon.data();
    buffer_beacon[1] = beacon.data();

    radioconfig_->radioStart(buffer_beacon.data());

#endif    

    thread_num_ = N_THREAD;
    /* initialize random seed: */
    srand (time(NULL));
    context = new PackageReceiverContext[thread_num_];

}

PackageReceiver::PackageReceiver(int N_THREAD, moodycamel::ConcurrentQueue<Event_data> * in_queue):
PackageReceiver(N_THREAD)
{
    message_queue_ = in_queue;
}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] context;
}

std::vector<pthread_t> PackageReceiver::startRecv(void** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == package_length * buffer_frame_num_); // should be integre
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    core_id_ = in_core_id;
    printf("start Recv thread\n");
    // new thread
    
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++)
    {
        pthread_t recv_thread_;
        // record the thread id 
        context[i].ptr = this;
        context[i].tid = i;
        // start socket thread
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
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id + tid) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
#endif
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    void* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

    void* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
#ifndef USE_SOCKET
    RadioConfig *radio = obj_ptr->radioconfig_;
    void* buffer2 = obj_ptr->buffer_[tid] + buffer_length;
    int* buffer_status2 = obj_ptr->buffer_status_[tid] + buffer_length;

    void* cur_ptr_buffer2;
    int* cur_ptr_buffer_status2;
    if (CH_NUM == 2)
    {
        cur_ptr_buffer2 = buffer2;
        cur_ptr_buffer_status2 = buffer_status2;
    }
    else
        cur_ptr_buffer2 = malloc(buffer_length); 
#endif
    // loop recv
    socklen_t addrlen = sizeof(obj_ptr->servaddr_);
    int offset = 0;
    int package_num = 0;
    long long frameTime;
    auto begin = std::chrono::system_clock::now();

    int maxQueueLength = 0;
    int ret = 0;
    while(true)
    {
        // if buffer is full, exit
        if(cur_ptr_buffer_status[0] == 1)
        {
            printf("thread %d buffer full\n", tid);
            exit(0);
        }
        int ant_id, frame_id, subframe_id, cell_id;
        // receive data
#ifdef USE_SOCKET
        int recvlen = -1;
        if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_, &addrlen)) < 0)
        {
            perror("recv failed");
            exit(0);
        }
        // read information from received packet
        frame_id = *((int *)cur_ptr_buffer);
        subframe_id = *((int *)cur_ptr_buffer + 1);
        cell_id = *((int *)cur_ptr_buffer + 2);
        ant_id = *((int *)cur_ptr_buffer + 3);
#else
        // this is probably a really bad implementation, and needs to be revamped
        void * samp1 = cur_ptr_buffer + 4*sizeof(int);
        void * samp2 = cur_ptr_buffer2 + 4*sizeof(int);
        void *samp[2] = {samp1, samp2};
        radio->radioRx(tid, samp, frameTime);
        frame_id = (int)(frameTime>>32);
        subframe_id = (int)(frameTime&0xFFFFFFFF);
        ant_id = tid;
        *((int *)cur_ptr_buffer) = frame_id;
        *((int *)cur_ptr_buffer + 1) = subframe_id;
        *((int *)cur_ptr_buffer + 2) = 0; //cell_id 
        *((int *)cur_ptr_buffer + 3) = ant_id;
        if (CH_NUM == 2)
        {
            *((int *)cur_ptr_buffer2) = frame_id;
            *((int *)cur_ptr_buffer2 + 1) = subframe_id;
            *((int *)cur_ptr_buffer2 + 2) = 0; //cell_id 
            *((int *)cur_ptr_buffer2 + 3) = ant_id + 1;
        }
#endif
        //printf("receive frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", frame_id, subframe_id, cell_id, ant_id);
        
        // get the position in buffer
        offset = cur_ptr_buffer_status - buffer_status;
        // move ptr & set status to full
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
        cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + 1) % buffer_frame_num;
        cur_ptr_buffer = buffer + ((char*)cur_ptr_buffer - (char*)buffer + package_length) % buffer_length;
        // push EVENT_PACKAGE_RECEIVED event into the queue
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_RECEIVED;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset + tid * buffer_frame_num;
        if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
#ifndef USE_SOCKET
        if (CH_NUM == 2)
        {
            offset = cur_ptr_buffer_status2 - buffer_status2;
            cur_ptr_buffer_status2[0] = 1; // has data, after doing fft, it is set to 0
            cur_ptr_buffer_status2 = buffer_status2 + (cur_ptr_buffer_status2 - buffer_status2 + 1) % buffer_frame_num;
            cur_ptr_buffer2 = buffer2 + ((char*)cur_ptr_buffer2 - (char*)buffer2 + package_length) % buffer_length;
            // push EVENT_PACKAGE_RECEIVED event into the queue
            Event_data package_message2;
            package_message2.event_type = EVENT_PACKAGE_RECEIVED;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            package_message2.data = offset + (tid + 1) * buffer_frame_num;
            if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
        }
#endif
        //printf("enqueue offset %d\n", offset);
        int cur_queue_len = message_queue_->size_approx();
        maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;

        package_num++;
        // print some information
        if(package_num == 1e5)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * 1e5;
            std::chrono::duration<double> diff = end - begin;
            // print network throughput & maximum message queue length during this period
            printf("thread %d receive %f bytes in %f secs, throughput %f MB/s, max Message Queue Length %d\n", tid, byte_len, diff.count(), byte_len / diff.count() / 1024 / 1024, maxQueueLength);
            maxQueueLength = 0;
            begin = std::chrono::system_clock::now();
            package_num = 0;
        }
    }
}

