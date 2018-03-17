#ifndef BUFFER_HEAD
#define BUFFER_HEAD

struct Event_data
{
    int event_type;
    int data;
};

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
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float ** FFT_inputs;
    complex_float ** FFT_outputs;
};

struct CSIBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector record CSI of BS_ANT_NUM * UE_NUM matrix
    std::vector<std::vector<complex_float>> CSI;
};

struct DataBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    std::vector<std::vector<complex_float>> data;
};

struct PrecoderBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector record CSI of UE_NUM * BS_ANT_NUM matrix
    std::vector<std::vector<complex_float>> precoder;
};

struct DemulBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    std::vector<std::vector<complex_float>> data;
};

#endif
