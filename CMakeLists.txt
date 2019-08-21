cmake_minimum_required(VERSION 3.14)
project(ina219)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_library(ina219 ina219.cc i2c.cc)