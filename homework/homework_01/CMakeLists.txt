cmake_minimum_required(VERSION 3.29)
project(AG1)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -pedantic -Wno-long-long -Werror -std=c++20)

add_executable(AG1 Room_217.cpp)
