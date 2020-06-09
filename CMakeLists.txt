cmake_minimum_required(VERSION 2.8.3)

include_directories(include)

add_library(adis_rcv_csv STATIC
  src/adis_rcv_csv.cpp)

#target_link_libraries(adis_rcv_csv ${catkin_LIBRARIES})

