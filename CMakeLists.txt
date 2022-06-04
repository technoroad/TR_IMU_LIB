cmake_minimum_required(VERSION 2.8.13)

include_directories(include)

add_library(adis_rcv_csv STATIC
  src/adis_rcv_csv.cpp)

