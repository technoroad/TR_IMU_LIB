project(adis_rcv_csv)
cmake_minimum_required(VERSION 3.5)

include_directories(include)

if (RUN_TEST)
  add_compile_definitions(UTEST)
endif()

add_library(adis_rcv_csv STATIC
  src/adis_rcv_csv.cpp)

if (RUN_TEST)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(test_adis_rcv_csv
    test/test_adis_rcv_csv.cpp
  )
  target_link_libraries(test_adis_rcv_csv
    GTest::gtest_main
    adis_rcv_csv
  )
  add_test(NAME test_adis_rcv_csv COMMAND test_adis_rcv_csv)

  enable_testing()
endif()