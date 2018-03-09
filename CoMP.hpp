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

#include "buffer.hpp"

#include "concurrentqueue.h"

class CoMP
{
public:
    static const int MAX_EPOLL_EVENTS_PER_RUN = 4;
    static const int TASK_THREAD_NUM = 4;
    static const int MAX_EVENT_NUM = TASK_THREAD_NUM + 1;

    static const int SOCKET_BUFFER_FRAME_NUM = 20; // buffer 10 frames
    static const int TASK_BUFFER_FRAME_NUM = 10;

    CoMP();
    ~CoMP();

    void start();
    static void* taskThread(void* context);
    void doCrop(int tid, int offset);
    void doZF(int tid, int offset);
    void doDemul(int tid, int offset);

    struct EventHandlerContext
    {
        CoMP* obj_ptr;
        int id;
    };

    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);
    inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id);

    inline bool isPilot(int subframe_id) {return (subframe_id >=0) && (subframe_id < UE_NUM); }
    inline bool isData(int subframe_id) {return (subframe_id < subframe_num_perframe) && (subframe_id >= UE_NUM); }

    inline complex_float divide(complex_float e1, complex_float e2);



private:
    std::unique_ptr<PackageReceiver> receiver_;
    SocketBuffer socket_buffer_;
    FFTBuffer fft_buffer_;
    CSIBuffer csi_buffer_;
    DataBuffer data_buffer_;
    PrecoderBuffer precoder_buffer_;

    std::vector<complex_float> pilots_;

    mufft_plan_1d* muplans_[TASK_THREAD_NUM];

    moodycamel::ConcurrentQueue<Event_data> task_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);
    moodycamel::ConcurrentQueue<Event_data> message_queue_ = moodycamel::ConcurrentQueue<Event_data>(SOCKET_BUFFER_FRAME_NUM * subframe_num_perframe * BS_ANT_NUM);

    bool task_status_[TASK_THREAD_NUM]; // can only be accessed in main_thread

    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    // all checkers
    int cropper_checker_[subframe_num_perframe * TASK_BUFFER_FRAME_NUM];
    int csi_checker_[TASK_BUFFER_FRAME_NUM];
    int data_checker_[TASK_BUFFER_FRAME_NUM];

    int precoder_checker_[TASK_BUFFER_FRAME_NUM];
    bool precoder_status_[TASK_BUFFER_FRAME_NUM];


    std::queue<std::tuple<int, int>> taskWaitList;

    int max_queue_delay = 0;

    int debug_count = 0;
};

#endif
