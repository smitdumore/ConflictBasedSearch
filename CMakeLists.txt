cmake_minimum_required(VERSION 3.5)

project(cbs)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")


# Include the header files directory
include_directories(include)

add_executable(run_cbs app/main.cpp)

# Link against required libraries
target_link_libraries(run_cbs
  ${Boost_LIBRARIES}
  yaml-cpp
)