#include "packageSender.hpp"

int main(int argc, char const *argv[])
{
    PackageSender sender;
    printf("send package\n");
    sender.loopSend();

    return 0;
}