cmake_minimum_required(VERSION 2.8.9)

project(test_timestamp_loopback)

find_package(SoapySDR REQUIRED)

if (SoapySDR_FOUND)
    message(STATUS "SoapySDR_INCLUDE_DIRS: ${SoapySDR_INCLUDE_DIRS}")
    message(STATUS "SoapySDR_LIBRARIES: ${SoapySDR_LIBRARIES}")
    message(STATUS "SoapySDR_DEFINITIONS: ${SoapySDR_DEFINITIONS}")
endif()

add_executable(test_timestamp_loopback test_timestamp_loopback.c)

target_link_libraries(test_timestamp_loopback ${SoapySDR_LIBRARIES})
