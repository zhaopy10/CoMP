#include "packageSender.hpp"


PackageSender::PackageSender():
frame_id(0), subframe_id(0)
{
    if ((socket_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("cannot create socket\n");
        exit(0);
    }
    buffer_.resize(BS_ANT_NUM);
    for(int i = 0; i < buffer_.size(); i++)
        buffer_[i].resize(buffer_length);

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

    IQ_data = new float[OFDM_FRAME_LEN * 2];
    //for (int k = OFDM_PREFIX_LEN * 2; k < OFDM_FRAME_LEN * 2; ++k)    
    //    IQ_data[k] = rand() / (float) RAND_MAX;
    //memcpy(IQ_data, IQ_data + (OFDM_FRAME_LEN - OFDM_PREFIX_LEN) * 2, sizeof(float) * OFDM_PREFIX_LEN * 2); // prefix

    // read from file
    FILE* fp = fopen("data.bin","rb");
    fread(IQ_data, sizeof(float), OFDM_FRAME_LEN * 2, fp);
    fclose(fp);

}

PackageSender::~PackageSender()
{
    delete[] IQ_data;
}

void PackageSender::genData()
{
    int cell_id = 0;
    
    for (int j = 0; j < buffer_.size(); ++j) // per antenna
    {
        memcpy(buffer_[j].data(), (char *)&frame_id, sizeof(int));
        memcpy(buffer_[j].data() + sizeof(int), (char *)&subframe_id, sizeof(int));
        memcpy(buffer_[j].data() + sizeof(int) * 2, (char *)&cell_id, sizeof(int));
        memcpy(buffer_[j].data() + sizeof(int) * 3, (char *)&j, sizeof(int));
        //printf("copy IQ\n");
        // waste some time
        for(int p = 0; p < 1e3; p++)
            rand();

        memcpy(buffer_[j].data() + data_offset, (char *)IQ_data, sizeof(float) * OFDM_FRAME_LEN * 2);   
    }

    subframe_id++;
    if(subframe_id == subframe_num_perframe)
    {
        subframe_id = 0;
        frame_id ++;
        if(frame_id == MAX_FRAME_ID)
            frame_id = 0;
    }
    
}

void PackageSender::loopSend()
{
    auto begin = std::chrono::system_clock::now();
    const int info_interval = 2e1;
    std::vector<int> ant_seq = std::vector<int>(buffer_.size());
    for (int i = 0; i < ant_seq.size(); ++i)
        ant_seq[i] = i;
    //std::iota(ant_seq.begin(), ant_seq.end(), 0);

    while(true)
    {
        this->genData(); // generate data
        //printf("genData\n");
        //std::random_shuffle ( ant_seq.begin(), ant_seq.end() ); // random perm
        for (int i = 0; i < buffer_.size(); ++i)
        {
            //usleep(1);
            /* send a message to the server */
            if (sendto(this->socket_, this->buffer_[ant_seq[i]].data(), this->buffer_length, 0, (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
                perror("socket sendto failed");
                exit(0);
            }
        }  
        //printf("send frame %d, subframe %d\n", frame_id, subframe_id);
        if ((frame_id+1) % info_interval == 0 && subframe_id == 0)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(float) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * subframe_num_perframe * info_interval;
            std::chrono::duration<double> diff = end - begin;
            printf("transmit %f bytes in %f secs, throughput %f MB/s\n", byte_len, diff.count(), byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
        }
    }
    
}