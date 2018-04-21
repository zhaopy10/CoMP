/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#ifndef COMP_HEAD
#define COMP_HEAD

#include "packageReceiver.hpp"
#include <unistd.h>
#include <memory>
#include <iostream>
#include <sys/epoll.h>
#include <fcntl.h>
#include <system_error>
#include <pthread.h>
#include <queue>
#include "mufft/fft.h"
#include <complex.h>
#include <math.h>
#include <tuple>
#include "cpu_attach.hpp"
#include <armadillo>
#include <immintrin.h>
#include "buffer.hpp"
#include "concurrentqueue.h"


class CoMP
{
public:
    // TASK & SOCKET thread number 
    static const int TASK_THREAD_NUM = 4;
#ifdef USE_SOCKET
    static const int SOCKET_THREAD_NUM = 7;
#else
    static const int SOCKET_THREAD_NUM = 1; // Set this RADIO_NUM
#endif 
    // buffer length of each socket thread
    // the actual length will be SOCKET_BUFFER_FRAME_NUM
    // * subframe_num_perframe * BS_ANT_NUM
    static const int SOCKET_BUFFER_FRAME_NUM = 80;
    // buffer length of computation part (for FFT/CSI/ZF/DEMUL buffers)
    static const int TASK_BUFFER_FRAME_NUM = 60;
    // do demul_block_size sub-carriers in each task
    static const int demul_block_size = 32;
    // optimization parameters for block transpose (see the slides for more
    // details)
    static const int transpose_block_size = 64;
    // dequeue bulk size, used to reduce the overhead of dequeue in main
    // thread
    static const int dequeue_bulk_size = 5;

    CoMP();
    ~CoMP();

    void start();
    // while loop of task thread
    static void* taskThread(void* context);
    // do different tasks
    void doCrop(int tid, int offset);
    void doZF(int tid, int offset);
    void doDemul(int tid, int offset);

    struct EventHandlerContext
    {
        CoMP* obj_ptr;
        int id;
    };
    // combine frame_id & subframe_id into one int
    inline int getSubframeBufferIndex(int frame_id, int subframe_id);
    inline void splitSubframeBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id);
    // combine frame_id & subframe_id & ant_id into one int
    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);
    inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id);
    
    inline bool isPilot(int subframe_id) {return subframe_id == 2;} //{return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return subframe_id == 3;}  //{return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }
    // complex divide
    inline complex_float divide(complex_float e1, complex_float e2);



private:
    std::unique_ptr<PackageReceiver> receiver_;
    SocketBuffer socket_buffer_[SOCKET_THREAD_NUM];
    FFTBuffer fft_buffer_;
    CSIBuffer csi_buffer_;
    DataBuffer data_buffer_;
    PrecoderBuffer precoder_buffer_;
    DemulBuffer demul_buffer_;

    std::vector<complex_float> pilots_;

    mufft_plan_1d* muplans_[TASK_THREAD_NUM];
   
    moodycamel::ConcurrentQueue<Event_data> task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    moodycamel::ConcurrentQueue<Event_data> message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM  * 36);
    
    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    // all checkers
    int cropper_checker_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];
    int csi_checker_[TASK_BUFFER_FRAME_NUM];
    int data_checker_[TASK_BUFFER_FRAME_NUM];

    int precoder_checker_[TASK_BUFFER_FRAME_NUM];
    bool precoder_status_[TASK_BUFFER_FRAME_NUM];

    int demul_checker_[TASK_BUFFER_FRAME_NUM][(subframe_num_perframe - UE_NUM)];

    std::queue<std::tuple<int, int>> taskWaitList;

    int max_queue_delay = 0;

    int debug_count = 0;

    std::unique_ptr<moodycamel::ProducerToken> task_ptok[TASK_THREAD_NUM];

    myVec spm_buffer[TASK_THREAD_NUM];
};

#endif
