cmake_minimum_required(VERSION 3.16.0)

list(APPEND EXTRA_COMPONENT_DIRS ".pio/libdeps/esp32-s3-devkitc-1-n16r8v/esp-idf-24c/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Lesson-4.8-miniproject)
