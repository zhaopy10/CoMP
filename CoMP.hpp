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

struct complex_float {
    float real;
    float imag;
};

struct SocketBuffer
{
    std::vector<char> buffer;
    std::vector<int> buffer_status;
};

struct FFTBuffer
{
    complex_float ** FFT_inputs;
    complex_float ** FFT_outputs;
};

//typename struct EventHandlerContext;

class CoMP
{
public:
    static const int MAX_EPOLL_EVENTS_PER_RUN = 1;
    static const int TASK_THREAD_NUM = 4;
    static const int MAX_EVENT_NUM = TASK_THREAD_NUM + 1;

    static const int SOCKET_BUFFER_FRAME_NUM = 10; // buffer 10 frames
    static const int TASK_BUFFER_FRAME_NUM = 2;

    CoMP();
    ~CoMP();

    void start();
    static void* taskThread(void* context);
    void doCrop(int tid, int offset);

    struct EventHandlerContext
    {
        CoMP* obj_ptr;
        int id;
    };

    inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id);

private:
    std::unique_ptr<PackageReceiver> receiver_;
    SocketBuffer socket_buffer_;
    FFTBuffer fft_buffer_;
    mufft_plan_1d* muplans_[TASK_THREAD_NUM];

    int epoll_fd;
    // A struct epoll_event for each process
    struct epoll_event event[MAX_EVENT_NUM];

    int pipe_socket_[2];
    int pipe_task_[TASK_THREAD_NUM][2]; // used to assign task
    int pipe_task_finish_[TASK_THREAD_NUM][2]; // used to inform main thread

    bool task_status_[TASK_THREAD_NUM]; // can only be accessed in main_thread

    int epoll_fd_task_side[TASK_THREAD_NUM];
    struct epoll_event event_task_side[TASK_THREAD_NUM];

    int cropper_checker_[subframe_num_perframe];

    pthread_t task_threads[TASK_THREAD_NUM];

    EventHandlerContext context[TASK_THREAD_NUM];

    std::queue<int> cropWaitList;

    int max_queue_delay = 0;
};


