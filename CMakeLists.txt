cmake_minimum_required (VERSION 2.6)
project (CoMP)

find_package(Armadillo)

#set(CMAKE_C_FLAGS "-std=c99 -w -mavx -mavx2  ")
#set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx -mavx2 -mavx512vl -march=native")
set(CMAKE_C_FLAGS "-std=c99 -w ")
set(CMAKE_CXX_FLAGS "-std=c++11 -w -O3 -mavx2 -mavx")


add_executable(sender sender_cli.cpp packageSender.cpp cpu_attach.cpp)
target_link_libraries(sender -lpthread)

add_executable(receiver receiver_cli.cpp packageReceiver.cpp cpu_attach.cpp )
target_link_libraries(receiver -lpthread)


add_executable(CoMP main.cpp packageReceiver.cpp CoMP.cpp cpu_attach.cpp)
#target_link_libraries(CoMP -llapack -lblas -lpthread /usr/local/lib/libmufft.a -larmadillo)
target_link_libraries(CoMP -lpthread /usr/local/lib/libmufft.a -larmadillo)


add_executable(trans testTranspose.cpp)
target_link_libraries(trans -larmadillo) 
