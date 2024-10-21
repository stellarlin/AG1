cmake_minimum_required(VERSION 3.25)
project(AG1)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -pedantic -Wno-long-long -Werror )

add_executable(AG1 Room_217.cpp)
