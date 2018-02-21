#include "CoMP.hpp"

CoMP::CoMP()
{
    // initialize socket buffer
    socket_buffer_.buffer.resize(PackageReceiver::package_length 
        * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM); // buffer entire frame
    socket_buffer_.buffer_status.resize(subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);

    // initialize FFT buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.FFT_inputs = new complex_float*[FFT_buffer_block_num];
    fft_buffer_.FFT_outputs = new complex_float*[FFT_buffer_block_num];
    for(int i = 0; i < FFT_buffer_block_num; i++)
    {
        fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        muplans_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);
    }


    // initialize pipes
    pipe(pipe_socket_);
    //fcntl(pipe_socket_[0], F_SETFL, fcntl(pipe_socket_[0], F_GETFL, 0) | O_NONBLOCK);           // non-blocking pipe
    //fcntl(pipe_socket_[1], F_SETFL, fcntl(pipe_socket_[1], F_GETFL, 0) | O_NONBLOCK);
    receiver_.reset(new PackageReceiver(pipe_socket_));

    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        pipe(pipe_task_[i]);
        pipe(pipe_task_finish_[i]);
        //fcntl(pipe_task_[i][0], F_SETFL, fcntl(pipe_task_[i][0], F_GETFL, 0) | O_NONBLOCK);           // non-blocking pipe
        //fcntl(pipe_task_[i][1], F_SETFL, fcntl(pipe_task_[i][1], F_GETFL, 0) | O_NONBLOCK);
    }

    
    epoll_fd = epoll_create(MAX_EVENT_NUM);
    // Setup the event 0 for package receiver
    event[0].events = EPOLLIN;
    event[0].data.fd = pipe_socket_[0];
    epoll_ctl( epoll_fd, EPOLL_CTL_ADD, pipe_socket_[0], &event[0]); 
    for(int i = 1; i < MAX_EVENT_NUM; i++)
    {
        event[i].events = EPOLLIN;
        event[i].data.fd = pipe_task_finish_[i-1][0];
        epoll_ctl( epoll_fd, EPOLL_CTL_ADD, pipe_task_finish_[i-1][0], &event[i]); 
    }

    // task side
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        epoll_fd_task_side[i] = epoll_create(1);
        event_task_side[i].events = EPOLLIN;
        event_task_side[i].data.fd = pipe_task_[i][0]; // port 0 at task threads
        epoll_ctl( epoll_fd_task_side[i], EPOLL_CTL_ADD, pipe_task_[i][0], &event_task_side[i]); 
    }
    
    memset(cropper_checker_, 0, sizeof(int) * subframe_num_perframe);
    memset(task_status_, 0, sizeof(bool) * TASK_THREAD_NUM); 

    // create
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::taskThread, &context[i]) != 0)
        {
            perror("task thread create failed");
            exit(0);
        }
    }
}

CoMP::~CoMP()
{
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        mufft_free_plan_1d(muplans_[i]);
    }
    // release FFT_buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    for(int i = 0; i < FFT_buffer_block_num; i++)
    {
        mufft_free(fft_buffer_.FFT_inputs[i]);
        mufft_free(fft_buffer_.FFT_outputs[i]);
    }
}

void CoMP::start()
{
    pthread_t recv_thread = receiver_->startRecv(socket_buffer_.buffer.data(), 
        socket_buffer_.buffer_status.data(), socket_buffer_.buffer_status.size(), socket_buffer_.buffer.size());
    // event loop
    struct epoll_event ev[MAX_EVENT_NUM];
    int nfds;
    while(true)
    {
        
        nfds = epoll_wait(epoll_fd, ev, MAX_EPOLL_EVENTS_PER_RUN, -1);
        if(nfds < 0)
        {
            std::error_code ec(errno, std::system_category());
            printf("Error in epoll_wait: %s\n", ec.message().c_str());
            exit(0);
        }
        
        // loop each ready events
        for(int i = 0; i < nfds; i++)
        {
            if(ev[i].data.fd == pipe_socket_[0]) // socket event
            {
                //printf("get socket event\n");
                int offset;
                read(pipe_socket_[0], (char *)&offset, sizeof(int)); // read position
                // find an empty thread and run
                int tid = -1;
                for (int i = 0; i < TASK_THREAD_NUM; ++i)
                {
                    if(!task_status_[i])
                    {
                        tid = i;
                        break;
                    }
                }
                
                if(tid >= 0) // find a thread
                {
                    // write TASK_CROP and offset
                    //printf("assign offset %d to thread %d\n", offset, tid);
                    char tmp_data[sizeof(int) * 2];
                    int task_id = TASK_CROP;
                    memcpy(tmp_data, &task_id, sizeof(int));
                    memcpy(tmp_data + sizeof(int), &(offset), sizeof(int));
                    write(pipe_task_[tid][1], tmp_data, sizeof(int) * 2); // port 1 at main_thread end
                    task_status_[tid] = true; // set status to busy
                }
                else // all thread are running
                {
                    cropWaitList.push(offset);
                }
                
            }
            else // task event
            {
                //printf("get task event\n");
                int tid = 0;
                while(ev[i].data.fd != pipe_task_finish_[tid][0]) // find tid
                {
                    tid++;
                }
                //printf("tid %d finished\n", tid);
                task_status_[tid] = false; // now free
                // the data length should be the same for all kinds of events ????
                char tmp_data[sizeof(int) * 2];
                read(pipe_task_finish_[tid][0], tmp_data, sizeof(int) * 2); // read 
                int event_id = *((int *)tmp_data);
                if(event_id == EVENT_CROPPED)
                {
                    int subframe_id = *((int *)tmp_data + 1);
                    // check subframe
                    cropper_checker_[subframe_id] ++;
                    if(cropper_checker_[subframe_id] == BS_ANT_NUM) // this sub-frame is finished
                    {
                        //printf("subframe_id %d finished\n", subframe_id);
                        cropper_checker_[subframe_id] = 0;
                        // TODO: assign further tasks
                    }
                    else
                    {
                        // find a task in the waiting list
                        if(cropWaitList.size() > 0) // not empty
                        {
                            if(cropWaitList.size() > max_queue_delay)
                            {
                                max_queue_delay = cropWaitList.size();
                                printf("maximum wait list length %d\n", cropWaitList.size());
                            }
                            
                            int offset = cropWaitList.front();
                            cropWaitList.pop();
                            char tmp_data_for_crop[sizeof(int) * 2];
                            int task_id_for_crop = TASK_CROP;
                            memcpy(tmp_data_for_crop, &task_id_for_crop, sizeof(int));
                            memcpy(tmp_data_for_crop + sizeof(int), &(offset), sizeof(int));
                            write(pipe_task_[tid][1], tmp_data_for_crop, sizeof(int) * 2); // port 1 at main_thread end
                        }
                    }
                }

                
            }
            
        }
        
        
        
    }
}

void* CoMP::taskThread(void* context)
{
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);

    int task_epoll_fd = obj_ptr->epoll_fd_task_side[tid];
    // wait for event
    struct epoll_event ev;
    char data_from_main[1024];
    int task_id;
    while(true)
    {
        //printf("thread %d wait for event\n", tid);
        int nfds = epoll_wait(task_epoll_fd, &ev, 1, -1);
        if(nfds < 0)
        {
            std::error_code ec(errno, std::system_category());
            printf("Error in thread %d, epoll_wait: %s\n", tid, ec.message().c_str());
            exit(0);
        }
        //printf("thread %d get event\n", tid);
        // the data length should be the same for all kinds of tasks????
        read(ev.data.fd, data_from_main, sizeof(int) * 2); 
        task_id = *((int *)data_from_main);
        if(task_id == TASK_CROP)
        {
            int offset;
            offset = *((int *)data_from_main + 1);
            //printf("thread %d task_id %d, offset %d\n", tid, task_id, offset);
            obj_ptr->doCrop(tid, offset);
        }
        else
        {

        }
    }
}

inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
}

void CoMP::doCrop(int tid, int offset)
{
    // read info
    char* cur_ptr_buffer = socket_buffer_.buffer.data() + offset * PackageReceiver::package_length;
    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    subframe_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
    //printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);
    memcpy((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id], 
        cur_ptr_buffer + sizeof(int) * 4 + (OFDM_PREFIX_LEN + delay_offset) * 2 * sizeof(float), 
        sizeof(float) * (OFDM_CA_NUM - delay_offset) * 2); // COPY
    if(delay_offset > 0) // append zero
    {
        memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
            + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    }
    // perform fft
    mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id], 
        fft_buffer_.FFT_inputs[FFT_buffer_target_id]);

    // debug
/*
    if(tid ==0)
    {
        complex_float *cur_fft_buffer_ptr = fft_buffer_.FFT_inputs[FFT_buffer_target_id];
        for(int i = 0; i < 10; i++)
            printf("%f + %f i\n", cur_fft_buffer_ptr[i].real, cur_fft_buffer_ptr[i].imag);
        printf("after fft\n");
        cur_fft_buffer_ptr = fft_buffer_.FFT_outputs[FFT_buffer_target_id];
        for(int i = 0; i < 10; i++)
            printf("%f + %f i\n", cur_fft_buffer_ptr[i].real, cur_fft_buffer_ptr[i].imag);
        exit(0);
    }
*/

    // after finish
    socket_buffer_.buffer_status[offset] = 0; // now empty
    // inform main thread
    char tmp_data[sizeof(int) * 2];
    int event_id = EVENT_CROPPED;
    memcpy(tmp_data, &event_id, sizeof(int));
    memcpy(tmp_data + sizeof(int), &(subframe_id), sizeof(int));

    write(pipe_task_finish_[tid][1], tmp_data, sizeof(int) * 2); // tell main thread crop finished
}