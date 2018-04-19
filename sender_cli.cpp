#include "packageSender.hpp"
#include "radio_lib.hpp"

void loadData(char* filename, std::vector<std::complex<int16_t>> &data, int samples)
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

void printVector(std::vector<std::complex<int16_t>> &data)
{
    for(int i = 0; i < data.size(); i++)
    {
        //printf("%f+%fj\t", data[i*2], data[i*2+1]);
        std::cout << real(data.at(i)) << " " << imag(data.at(i)) << std::endl;
    }
}

int main(int argc, char const *argv[])
{
	/*
    if(argc > 3)
        PackageSender sender(strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10), strtol(argv[3], NULL, 10));
    else
        PackageSender sender(strtol(argv[1], NULL, 10), strtol(argv[2], NULL, 10));
    printf("send package\n");
    */
    std::vector<std::string> bs_radio_ids;
    bs_radio_ids.push_back("0378");
    int maxFrame = 1 << 31;
    int symnum = 20;
    int symlen = 256;
    std::vector<int> tdd_sched(symnum);
    std::fill(tdd_sched.begin(), tdd_sched.end(), 0);
    tdd_sched[0] = 1;
    tdd_sched[2] = 2;
    tdd_sched[3] = 2;
    RadioConfig radioConfig(bs_radio_ids, 2, 5e6, 915e6, 20, 50, symlen, symnum, maxFrame, tdd_sched);

    std::vector<std::complex<int16_t>> beacon;
    loadData("beacon.txt", beacon, symlen);
    printVector(beacon);
    std::vector<void * > buffer_beacon(2);
    buffer_beacon[0] = beacon.data();
    buffer_beacon[1] = beacon.data();

    radioConfig.radioStart(buffer_beacon.data());


    //std::vector<float> wb_pilot;
    //loadData("lts.txt", wb_pilot, symlen);
    //std::vector<void * > buffer_txData;
    //buffer_beacon.push_back(beacon.data());
    while (true);    

    return 0;
}
