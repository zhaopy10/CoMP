#include "packageReceiver.hpp"

int main(int argc, char const *argv[])
{
    PackageReceiver receiver;

    // test
    std::vector<char> buffer(PackageReceiver::package_length * PackageReceiver::subframe_num_perframe * BS_ANT_NUM );
    printf("buffer len %d\n", buffer.size());
    std::vector<int> buffer_status(PackageReceiver::subframe_num_perframe * BS_ANT_NUM);
    std::fill(buffer_status.begin(), buffer_status.end(), 0);

    pthread_t recv_thread = receiver.startRecv(buffer.data(), buffer_status.data(), buffer_status.size(), buffer.size());
    pthread_join(recv_thread, NULL);

    return 0;
}