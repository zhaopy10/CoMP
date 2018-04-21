cmake_minimum_required (VERSION 2.8)
project (CoMP)

find_package(Armadillo)

set(CMAKE_C_FLAGS "-std=c99 -w ")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx")

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
add_definitions(-DUSE_SOCKET)

add_executable(sender sender_cli.cpp packageSender.cpp cpu_attach.cpp radio_lib.cpp)
target_link_libraries(sender -lpthread ${SoapySDR_LIBRARIES})

add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp radio_lib.cpp)
target_link_libraries(receiver -lpthread ${SoapySDR_LIBRARIES})


add_executable(CoMP main.cpp packageReceiver.cpp CoMP.cpp cpu_attach.cpp radio_lib.cpp)
target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo ${SoapySDR_LIBRARIES})

add_library(mubf SHARED packageReceiver.cpp CoMP.cpp cpu_attach.cpp radio_lib.cpp)
target_link_libraries(mubf -lpthread /usr/local/lib/libmufft.so -larmadillo ${SoapySDR_LIBRARIES}) 


add_executable(trans testTranspose.cpp)
target_link_libraries(trans -larmadillo) 

