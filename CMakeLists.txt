cmake_minimum_required (VERSION 2.8)
project (CoMP)

find_package(Armadillo)
#set(CMAKE_C_FLAGS "-std=gnu99 -w -mavx -mavx2 -msse2 ")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx -mavx2 -msse2")

########################################################################
# SoapySDR dependency
########################################################################
find_package(SoapySDR "0.6" CONFIG)

if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "SoapySDR development files not found")
    return()
endif ()


message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
include_directories(${SoapySDR_INCLUDE_DIRS})
add_executable(sender sender_cli.cpp packageSender.cpp radio_lib.cpp)
target_link_libraries(sender -lpthread ${SoapySDR_LIBRARIES})

add_executable(CoMP main.cpp packageReceiver.cpp CoMP.cpp)
target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.so)
