# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Admin/Downloads/app/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "C:/lab project 1/water_level_project/build/bootloader"
  "C:/lab project 1/water_level_project/build/bootloader-prefix"
  "C:/lab project 1/water_level_project/build/bootloader-prefix/tmp"
  "C:/lab project 1/water_level_project/build/bootloader-prefix/src/bootloader-stamp"
  "C:/lab project 1/water_level_project/build/bootloader-prefix/src"
  "C:/lab project 1/water_level_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/lab project 1/water_level_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/lab project 1/water_level_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
