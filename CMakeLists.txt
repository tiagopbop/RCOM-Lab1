cmake_minimum_required(VERSION 3.29)
project(RCOM_Lab1)

set(CMAKE_CXX_STANDARD 14)

include_directories(LAB1/include)

add_executable(RCOM_Lab1
        LAB1/bin/cable
        LAB1/bin/main
        LAB1/cable/cable.c
        LAB1/include/application_layer.h
        LAB1/include/link_layer.h
        LAB1/include/serial_port.h
        LAB1/src/application_layer.c
        LAB1/src/link_layer.c
        LAB1/src/serial_port.c
        LAB1/main.c
        LAB1/Makefile)
