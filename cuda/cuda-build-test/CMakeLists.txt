#Be sure you have win64 generator selected if building on windows
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(cuda_build_test LANGUAGES CXX CUDA)

add_executable(test src/test.cu)
set_property(TARGET test 
             PROPERTY CUDA_SEPARABLE_COMPILATION ON)