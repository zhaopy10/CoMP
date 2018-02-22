#ifndef SYMBOLS
#define SYMBOLS

#define BS_ANT_NUM 96
#define OFDM_CA_NUM 1024
#define OFDM_PREFIX_LEN 128
#define UE_NUM 4

#define EVENT_PACKAGE_RECEIVED 0
#define EVENT_CROPPED 1

#define TASK_CROP 0


static const int subframe_num_perframe = 40;
static const int pilot_subframe_num_perframe = UE_NUM;
static const int data_subframe_num_perframe = subframe_num_perframe - pilot_subframe_num_perframe;

static const int MAX_FRAME_ID = 1e4;

#endif
