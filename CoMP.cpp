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
            * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
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
    // initialize muplans for fft
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

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        spm_buffer[i].resize(BS_ANT_NUM);

    // read pilots from file
    pilots_.resize(OFDM_CA_NUM);
    FILE* fp = fopen("pilots.bin","rb");
    fread(pilots_.data(), sizeof(float), OFDM_CA_NUM * 2, fp);
    fclose(fp);

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_THREAD_NUM, &message_queue_));

    // initilize all kinds of checkers
    memset(cropper_checker_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_status_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 
    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
    {
        memset(demul_checker_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));
    }

    // create task thread
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
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(0) != 0)
    {
        perror("stitch main thread to core 0 failed");
        exit(0);
    }
#endif
    // creare socket buffer and socket threads
    void* socket_buffer_ptrs[SOCKET_THREAD_NUM];
    int* socket_buffer_status_ptrs[SOCKET_THREAD_NUM];
    for(int i = 0; i < SOCKET_THREAD_NUM; i++)
    {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> recv_thread = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);
    // for task_queue, main thread is producer, it is single-procuder & multiple consumer
    // for task queue
    moodycamel::ProducerToken ptok(task_queue_);
    // for message_queue, main thread is a comsumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while(true)
    {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e7)
        {
            // print the message_queue_ miss rate is needed
            //printf("message dequeue miss rate %f\n", (float)miss_count / total_count);
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0)
        {
            miss_count++;
            continue;
        }
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++)
        {
            Event_data& event = events_list[bulk_count];

            // if EVENT_PACKAGE_RECEIVED, do crop
            if(event.event_type == EVENT_PACKAGE_RECEIVED)
            {
                int offset = event.data;
                Event_data do_crop_task;
                do_crop_task.event_type = TASK_CROP;
                do_crop_task.data = offset;
                if ( !task_queue_.try_enqueue(ptok, do_crop_task ) ) {
                    printf("need more memory\n");
                    if ( !task_queue_.enqueue(ptok, do_crop_task ) ) {
                        printf("crop task enqueue failed\n");
                        exit(0);
                    }
                }

                
            }
            else if(event.event_type == EVENT_CROPPED)
            {
                // if EVENT_CROPPED, check if all antennas are ready, do ZF if
                // pilot, do deMul if data
                int FFT_buffer_target_id = event.data;
                // get frame_id & subframe_id
                int frame_id, subframe_id, ant_id;
                splitSubframeBufferIndex(FFT_buffer_target_id, &frame_id, &subframe_id);
                // check if all antennas in this subframe is ready
                int cropper_checker_id = frame_id * subframe_num_perframe + subframe_id;
                cropper_checker_[cropper_checker_id] ++;

                if(cropper_checker_[cropper_checker_id] == BS_ANT_NUM) // this sub-frame is finished
                {
                    //printf("subframe %d, frame %d\n", subframe_id, frame_id);
                    
                    cropper_checker_[cropper_checker_id] = 0; // this subframe is finished
                    // if this subframe is pilot part
                    if(isPilot(subframe_id))
                    {   
                        // check if csi of all UEs are ready
                        int csi_checker_id = frame_id;
                        csi_checker_[csi_checker_id] ++;
                        if(csi_checker_[csi_checker_id] == UE_NUM)
                        {
                            // if CSI from all UEs are ready, do ZF
                            csi_checker_[csi_checker_id] = 0;
                            //if(frame_id == 4)
                            //    printf("Frame %d, csi ready, assign ZF\n", frame_id);
                            
                            Event_data do_ZF_task;
                            do_ZF_task.event_type = TASK_ZF;
                            // add ZF tasks for each sub-carrier
                            for(int i = 0; i < OFDM_CA_NUM; i++)
                            {
                                int csi_offset_id = frame_id * OFDM_CA_NUM + i;
                                do_ZF_task.data = csi_offset_id;
                                if ( !task_queue_.try_enqueue(ptok, do_ZF_task ) ) {
                                    printf("need more memory\n");
                                    if ( !task_queue_.enqueue(ptok, do_ZF_task ) ) {
                                        printf("ZF task enqueue failed\n");
                                        exit(0);
                                    }
                                }
                            }
                        }
                    }
                    else if(isData(subframe_id))
                    {
                        // if this subframe is data part
                        int data_checker_id = frame_id;
                        data_checker_[data_checker_id] ++;
                        if(data_checker_[data_checker_id] == data_subframe_num_perframe)
                        {
                            // if all data part is ready. TODO: optimize the
                            // logic, do deMul when CSI is ready, do not wait
                            // the entire frame is ready
                            
                            //printf("do demul for frame %d\n", frame_id);
                            //printf("frame %d data received\n", frame_id);
                            data_checker_[data_checker_id] = 0;
                            Event_data do_demul_task;
                            do_demul_task.event_type = TASK_DEMUL;
                            // add deMul tasks
                            for(int j = 0; j < data_subframe_num_perframe; j++)
                            {
                                for(int i = 0; i < OFDM_CA_NUM / demul_block_size; i++)
                                {
                                    int demul_offset_id = frame_id * OFDM_CA_NUM * data_subframe_num_perframe
                                        + j * OFDM_CA_NUM + i * demul_block_size;
                                    do_demul_task.data = demul_offset_id;
                                    if ( !task_queue_.try_enqueue(ptok, do_demul_task ) ) {
                                        printf("need more memory\n");
                                        if ( !task_queue_.enqueue(ptok, do_demul_task ) ) {
                                            printf("Demultiplexing task enqueue failed\n");
                                            exit(0);
                                        }
                                    }
                                }
                            }
                        }     
                    }
                }
            }
            else if(event.event_type == EVENT_ZF)
            {
                // if ZF is ready 
                int offset_zf = event.data;
                // precoder is ready, do demodulation
                int ca_id = offset_zf % OFDM_CA_NUM;
                int frame_id = (offset_zf - ca_id) / OFDM_CA_NUM;
                precoder_checker_[frame_id] ++;
                if(precoder_checker_[frame_id] == OFDM_CA_NUM)
                {
                    precoder_checker_[frame_id] = 0;
                    //TODO: this flag can be used to optimize deDemul logic
                    precoder_status_[frame_id] = true;
                }
            }
            else if(event.event_type == EVENT_DEMUL)
            {
                // do nothing
                int offset_demul = event.data;
                int ca_id = offset_demul % OFDM_CA_NUM;
                int offset_without_ca = (offset_demul - ca_id) / OFDM_CA_NUM;
                int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
                int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);
                demul_checker_[frame_id][data_subframe_id] += demul_block_size;
                // if this subframe is ready
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
                    // print log per 100 frames
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

    
}

void* CoMP::taskThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
    // Note: cores 0-17, 36-53 are on the same socket
#ifdef ENABLE_CPU_ATTACH
    int offset_id = SOCKET_THREAD_NUM + 1;
    int tar_core_id = tid + offset_id;
    if(tar_core_id >= 18)
        tar_core_id = (tar_core_id - 18) + 36;
    if(stick_this_thread_to_core(tar_core_id) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->message_queue_));

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;
    while(true)
    {
        ret = task_queue_->try_dequeue(event);
        if(tid == 0)
            total_count++;
        if(tid == 0 && total_count == 1e6)
        {
            // print the task queue miss rate if required
            //printf("thread 0 task dequeue miss rate %f, queue length %d\n", (float)miss_count / total_count, task_queue_->size_approx());
            total_count = 0;
            miss_count = 0;
        }
        if(!ret)
        {
            if(tid == 0)
                miss_count++;
            continue;
        }

        // do different tasks according to task type
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
}

inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
}

inline int CoMP::getSubframeBufferIndex(int frame_id, int subframe_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * subframe_num_perframe + subframe_id;
}

inline void CoMP::splitSubframeBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id)
{
    (*frame_id) = FFT_buffer_target_id / subframe_num_perframe;
    (*subframe_id) = FFT_buffer_target_id - (*frame_id) * subframe_num_perframe;
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
    // get information 
    int ca_id = offset % OFDM_CA_NUM;
    int offset_without_ca = (offset - ca_id) / OFDM_CA_NUM;
    int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
    int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);

    //printf("do demul, %d %d %d\n", frame_id, data_subframe_id, ca_id);
    // see the slides for more details
    __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);
    float* tar_ptr = (float *)&data_buffer_.data[offset_without_ca][0];
    float* temp_buffer_ptr = (float *)&spm_buffer[tid][0];
    for(int i = 0; i < demul_block_size; i++)
    {
        for(int c2 = 0; c2 < BS_ANT_NUM / 4; c2++)
        {
            
            int c1_base = (ca_id + i) * 2 / transpose_block_size;
            int c1_offset = (ca_id + i) * 2 % transpose_block_size;
            float* base_ptr = tar_ptr + c1_base * transpose_block_size * BS_ANT_NUM + c1_offset + c2 * transpose_block_size * 4;
            __m256d t_data = _mm256_i64gather_pd((double*)base_ptr, index, 8);
            _mm256_store_pd((double*)(temp_buffer_ptr + c2 * 8), t_data);
        }

        int precoder_offset = frame_id * OFDM_CA_NUM + ca_id + i;
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset].data();
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        cx_float* data_ptr = (cx_float *)(&spm_buffer[tid][0]);
        cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

        cx_float* demul_ptr = (cx_float *)(&demul_buffer_.data[offset_without_ca][(ca_id + i) * UE_NUM]);
        cx_fmat mat_demuled(demul_ptr, UE_NUM, 1, false);

        mat_demuled = mat_precoder * mat_data;
/*
        //debug
        if(ca_id == 640 && i == 9 && frame_id == 4 && data_subframe_id == 0)
        {
            printf("thread %d, save mat precoder\n", tid);
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

    }

    // inform main thread
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    
    if ( !message_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }
    
    //printf("put demul event\n");
}

// do ZF
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

    if ( !message_queue_.enqueue(*task_ptok[tid], ZF_finish_event ) ) {
        printf("ZF message enqueue failed\n");
        exit(0);
    }
}

// do Crop
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
        
        /* //naive transpose
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        */
        // block transpose
        float* src_ptr = (float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        float* tar_ptr = (float *)&data_buffer_.data[frame_offset][0];
        for(int c2 = 0; c2 < OFDM_CA_NUM / transpose_block_size * 2; c2++)
        {
            for(int c3 = 0; c3 < transpose_block_size / 8; c3++)
            {
                __m256 data = _mm256_load_ps(src_ptr + c2 * transpose_block_size + c3 * 8);
                _mm256_store_ps(tar_ptr + c2 * BS_ANT_NUM * transpose_block_size + transpose_block_size * ant_id + c3 * 8, data);
            }
            
        }
        
    }


    // after finish
    socket_buffer_[buffer_id].buffer_status[offset] = 0; // now empty
    // inform main thread
    Event_data crop_finish_event;
    crop_finish_event.event_type = EVENT_CROPPED;
    crop_finish_event.data = getSubframeBufferIndex(frame_id, subframe_id);

    if ( !message_queue_.enqueue(*task_ptok[tid], crop_finish_event ) ) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}

