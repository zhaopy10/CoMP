#include "CoMP.hpp"

using namespace arma;
typedef cx_float COMPLEX;

CoMP::CoMP()
{
    printf("enter constructor\n");
    // initialize socket buffer
    for(int i = 0; i < SOCKET_THREAD_NUM; i++)
    {
        socket_buffer_[i].buffer.resize(PackageReceiver::package_length 
            * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM); // buffer entire frame
        socket_buffer_[i].buffer_status.resize(subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);
    }
    printf("initialize buffers\n");
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
    // initialize CSI buffer
    csi_buffer_.CSI.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < csi_buffer_.CSI.size(); i++)
        csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);

    // initialize data buffer
    data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < data_buffer_.data.size(); i++)
        data_buffer_.data[i].resize(BS_ANT_NUM * OFDM_CA_NUM);

    // initialize precoder buffer
    precoder_buffer_.precoder.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < precoder_buffer_.precoder.size(); i++)
        precoder_buffer_.precoder[i].resize(UE_NUM * BS_ANT_NUM);

    // initialize demultiplexed data buffer
    demul_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < demul_buffer_.data.size(); i++)
        demul_buffer_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    // read pilots from file
    pilots_.resize(OFDM_CA_NUM);
    FILE* fp = fopen("pilots.bin","rb");
    fread(pilots_.data(), sizeof(float), OFDM_CA_NUM * 2, fp);
    fclose(fp);

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_THREAD_NUM, &message_queue_));

    
    memset(cropper_checker_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(task_status_, 0, sizeof(bool) * TASK_THREAD_NUM); 
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_status_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 
    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
    {
        memset(demul_checker_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));
    }

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
    
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(0) != 0)
    {
        perror("stitch main thread to core 0 failed");
        exit(0);
    }
#endif
    // attach to core 1, but if ENABLE_CPU_ATTACH is not defined, it does not work
    char* socket_buffer_ptrs[SOCKET_THREAD_NUM];
    int* socket_buffer_status_ptrs[SOCKET_THREAD_NUM];
    for(int i = 0; i < SOCKET_THREAD_NUM; i++)
    {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> recv_thread = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);

    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();

    Event_data event;
    bool ret = false;
    while(true)
    {
        // get an event
        ret = message_queue_.try_dequeue(event);
        if(!ret)
            continue;

        

        // according to the event type
        if(event.event_type == EVENT_PACKAGE_RECEIVED)
        {
            int offset = event.data;
            Event_data do_crop_task;
            do_crop_task.event_type = TASK_CROP;
            do_crop_task.data = offset;
            if ( !task_queue_.enqueue( do_crop_task ) ) {
                printf("crop task enqueue failed\n");
                exit(0);
            }

            
        }
        else if(event.event_type == EVENT_CROPPED)
        {
            int FFT_buffer_target_id = event.data;
            // check subframe
            int frame_id, subframe_id, ant_id;
            splitFFTBufferIndex(FFT_buffer_target_id, &frame_id, &subframe_id, &ant_id);
                   
            int cropper_checker_id = frame_id * subframe_num_perframe + subframe_id;
            cropper_checker_[cropper_checker_id] ++;

            if(cropper_checker_[cropper_checker_id] == BS_ANT_NUM) // this sub-frame is finished
            {
                cropper_checker_[cropper_checker_id] = 0; // this subframe is finished
                if(isPilot(subframe_id))
                {
                    int csi_checker_id = frame_id;
                    csi_checker_[csi_checker_id] ++;
                    if(csi_checker_[csi_checker_id] == UE_NUM)
                    {
                        csi_checker_[csi_checker_id] = 0;
                        //if(frame_id == 4)
                        //    printf("Frame %d, csi ready, assign ZF\n", frame_id);
                        
                        Event_data do_ZF_task;
                        do_ZF_task.event_type = TASK_ZF;
                        for(int i = 0; i < OFDM_CA_NUM; i++)
                        {
                            int csi_offset_id = frame_id * OFDM_CA_NUM + i;
                            do_ZF_task.data = csi_offset_id;
                            if ( !task_queue_.enqueue( do_ZF_task ) ) {
                                printf("ZF task enqueue failed\n");
                                exit(0);
                            }
                        }
                    }
                }
                else if(isData(subframe_id))
                {
                    int data_checker_id = frame_id;
                    data_checker_[data_checker_id] ++;
                    if(data_checker_[data_checker_id] == data_subframe_num_perframe)
                    {
                        //printf("do demul for frame %d\n", frame_id);
                        //printf("frame %d data received\n", frame_id);
                        data_checker_[data_checker_id] = 0;
                        // just forget check, and optimize it later
                        Event_data do_demul_task;
                        do_demul_task.event_type = TASK_DEMUL;
                        for(int j = 0; j < data_subframe_num_perframe; j++)
                        {
                            for(int i = 0; i < OFDM_CA_NUM; i++)
                            {
                                int demul_offset_id = frame_id * OFDM_CA_NUM * data_subframe_num_perframe
                                    + j * OFDM_CA_NUM + i;
                                do_demul_task.data = demul_offset_id;
                                if ( !task_queue_.enqueue( do_demul_task ) ) {
                                    printf("Demuliplexing task enqueue failed\n");
                                    exit(0);
                                }
                            }
                        }
                    }     
                }
            }
        }
        else if(event.event_type == EVENT_ZF)
        {
            //printf("get ZF event\n");
            int offset_zf = event.data;
            // precoder is ready, do demodulation
            int ca_id = offset_zf % OFDM_CA_NUM;
            int frame_id = (offset_zf - ca_id) / OFDM_CA_NUM;
            precoder_checker_[frame_id] ++;
            if(precoder_checker_[frame_id] == OFDM_CA_NUM)
            {
                precoder_checker_[frame_id] = 0;
                precoder_status_[frame_id] = true;
                //if(frame_id == 0)
                //    printf("frame %d precoder ready\n", frame_id);
            }
        }
        else if(event.event_type == EVENT_DEMUL)
        {
            //printf("get demul event\n");
            // do nothing
            int offset_demul = event.data;
            int ca_id = offset_demul % OFDM_CA_NUM;
            int offset_without_ca = (offset_demul - ca_id) / OFDM_CA_NUM;
            int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
            int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);
            demul_checker_[frame_id][data_subframe_id] ++;
            if(demul_checker_[frame_id][data_subframe_id] == OFDM_CA_NUM)
            {
                demul_checker_[frame_id][data_subframe_id] = 0;
                
                // debug
                if(frame_id == 4 && data_subframe_id == 4)
                {
                    printf("save demul data\n");
                    FILE* fp = fopen("ver_data.txt","w");
                    for(int cc = 0; cc < OFDM_CA_NUM; cc++)
                    {
                        complex_float* cx = &demul_buffer_.data[offset_without_ca][cc * UE_NUM];
                        for(int kk = 0; kk < UE_NUM; kk++)  
                            fprintf(fp, "%f %f\n", cx[kk].real, cx[kk].imag);
                    }
                    fclose(fp);
                    exit(0);
                }
                

                demul_count += 1;
                if(demul_count == data_subframe_num_perframe * 100)
                {
                    demul_count = 0;
                    auto demul_end = std::chrono::system_clock::now();
                    std::chrono::duration<double> diff = demul_end - demul_begin;
                    int samples_num_per_UE = OFDM_CA_NUM * data_subframe_num_perframe * 100;
                    printf("Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %d\n", 
                        samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), task_queue_.size_approx());
                    demul_begin = std::chrono::system_clock::now();
                }
            }
        }
    }

    
}

void* CoMP::taskThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(tid + SOCKET_THREAD_NUM + 1) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, tid + 1);
        exit(0);
    }
#endif

    Event_data event;
    bool ret = false;
    while(true)
    {
        ret = task_queue_->try_dequeue(event);
        if(!ret)
            continue;
        if(event.event_type == TASK_CROP)
        {   
            obj_ptr->doCrop(tid, event.data);
        }
        else if(event.event_type == TASK_ZF)
        {
            obj_ptr->doZF(tid, event.data);
        }
        else if(event.event_type == TASK_DEMUL)
        {
            obj_ptr->doDemul(tid, event.data);
        }
        
    }

    /*

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
        int offset;
        offset = *((int *)data_from_main + 1);
        if(task_id == TASK_CROP)
        {   
            obj_ptr->doCrop(tid, offset);
        }
        else if(task_id == TASK_ZF)
        {
            obj_ptr->doZF(tid, offset);
        }
        else
        {

        }
    }
    */
}

inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
}

inline void CoMP::splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id)
{
    (*frame_id) = FFT_buffer_target_id / (BS_ANT_NUM * subframe_num_perframe);
    FFT_buffer_target_id = FFT_buffer_target_id - (*frame_id) * (BS_ANT_NUM * subframe_num_perframe);
    (*subframe_id) = FFT_buffer_target_id / BS_ANT_NUM;
    (*ant_id) = FFT_buffer_target_id - *subframe_id * BS_ANT_NUM;
}

inline complex_float CoMP::divide(complex_float e1, complex_float e2)
{
    complex_float re;
    float module = e2.real * e2.real + e2.imag * e2.imag;
    re.real = (e1.real * e2.real + e1.imag * e2.imag) / module;
    re.imag = (e1.imag * e2.real - e1.real * e2.imag) / module;
    return re;
}

void CoMP::doDemul(int tid, int offset)
{
    
    //int demul_offset_id = frame_id * OFDM_CA_NUM * (subframe_num_perframe - UE_NUM)
    //                                + j * OFDM_CA_NUM + i;
    int ca_id = offset % OFDM_CA_NUM;
    int offset_without_ca = (offset - ca_id) / OFDM_CA_NUM;
    int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
    int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);

    //printf("do demul, %d %d %d\n", frame_id, data_subframe_id, ca_id);

    int precoder_offset = frame_id * OFDM_CA_NUM + ca_id;
    cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset].data();
    cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

    cx_float* data_ptr = (cx_float *)(&data_buffer_.data[offset_without_ca][ca_id * BS_ANT_NUM]);
    cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

    cx_float* demul_ptr = (cx_float *)(&demul_buffer_.data[offset_without_ca][ca_id * UE_NUM]);
    cx_fmat mat_demuled(demul_ptr, UE_NUM, 1, false);

    mat_demuled = mat_precoder * mat_data;

    /*
    //debug
    if(ca_id == 2 && frame_id == 4)
    {
        printf("save mat precoder\n");
        FILE* fp_debug = fopen("tmpPrecoder.txt", "w");
        for(int i = 0; i < UE_NUM; i++)
        {
            for(int j = 0; j < BS_ANT_NUM; j++)
                fprintf(fp_debug, "%f %f ", mat_precoder.at(i,j).real(), mat_precoder.at(i,j).imag());
            fprintf(fp_debug, "\n" );
        }
        fclose(fp_debug);

        fp_debug = fopen("tmpData.txt","w");
        for(int i = 0; i < BS_ANT_NUM; i++)
            fprintf(fp_debug, "%f %f\n", mat_data.at(i,0).real(), mat_data.at(i,0).imag());
        fclose(fp_debug);

        fp_debug = fopen("tmpDemul.txt","w");
        for(int i = 0; i < UE_NUM; i++)
            fprintf(fp_debug, "%f %f\n", mat_demuled.at(i,0).real(), mat_demuled.at(i,0).imag());
        fclose(fp_debug);
        
        exit(0);
    }
    */

    // inform main thread
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;

    if ( !message_queue_.enqueue( demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }
    //printf("put demul event\n");
}


void CoMP::doZF(int tid, int offset)
{

    int ca_id = offset % OFDM_CA_NUM;
    int frame_id = (offset - ca_id) / OFDM_CA_NUM;


    cx_float* ptr_in = (cx_float *)csi_buffer_.CSI[offset].data();
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    cx_float* ptr_out = (cx_float *)precoder_buffer_.precoder[offset].data();
    cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
    
    pinv(mat_output, mat_input, 1e-1, "dc");


    // inform main thread
    Event_data ZF_finish_event;
    ZF_finish_event.event_type = EVENT_ZF;
    ZF_finish_event.data = offset;

    if ( !message_queue_.enqueue( ZF_finish_event ) ) {
        printf("ZF message enqueue failed\n");
        exit(0);
    }
}

void CoMP::doCrop(int tid, int offset)
{
    int buffer_frame_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    int buffer_id = offset / buffer_frame_num;
    offset = offset - buffer_id * buffer_frame_num;
    // read info
    char* cur_ptr_buffer = socket_buffer_[buffer_id].buffer.data() + offset * PackageReceiver::package_length;
    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    subframe_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
    //printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);

    // transfer ushort to float
    ushort* cur_ptr_buffer_ushort = (ushort*)(cur_ptr_buffer + sizeof(int) * 4);
    float* cur_fft_buffer_float = (float*)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
        cur_fft_buffer_float[i] = (cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] / 65536.f - 0.5f) * 4.f;
    //memcpy((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id], 
    //    cur_ptr_buffer + sizeof(int) * 4 + (OFDM_PREFIX_LEN + delay_offset) * 2 * sizeof(float), 
    //    sizeof(float) * (OFDM_CA_NUM - delay_offset) * 2); // COPY
    if(delay_offset > 0) // append zero
    {
        memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
            + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    }
    // perform fft
    mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id], 
        fft_buffer_.FFT_inputs[FFT_buffer_target_id]);

    // if it is pilot part, do CE
    if(isPilot(subframe_id))
    {
        int UE_id = subframe_id;
        int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            csi_buffer_.CSI[ca_offset + j][csi_offset] = divide(fft_buffer_.FFT_outputs[FFT_buffer_target_id][j], pilots_[j]);
        }       
    }
    else if(isData(subframe_id)) // if it is data part, just transpose
    {
        
        int data_subframe_id = subframe_id - UE_NUM;
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + data_subframe_id;
        
        
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        
    }


    // after finish
    socket_buffer_[buffer_id].buffer_status[offset] = 0; // now empty
    // inform main thread
    Event_data crop_finish_event;
    crop_finish_event.event_type = EVENT_CROPPED;
    crop_finish_event.data = FFT_buffer_target_id;

    if ( !message_queue_.enqueue( crop_finish_event ) ) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}

