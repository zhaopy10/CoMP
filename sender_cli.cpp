#include "packageSender.hpp"

int main(int argc, char const *argv[])
{
    PackageSender sender(strtol(argv[1], NULL, 10));
    printf("send package\n");
    sender.loopSend();

    return 0;
}