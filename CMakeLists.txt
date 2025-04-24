cmake_minimum_required(VERSION 3.10)
project(cbs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile warnings
add_compile_options(-Wall -Wextra)

# Dependencies
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_package(SFML 2.5 COMPONENTS system window graphics)
find_package(Threads REQUIRED)

# Header files
set(HEADER_FILES
    include/cbs/simulator.hpp
    include/cbs/common.hpp
    include/cbs/controller.hpp
)

# Executables
add_executable(run_cbs src/cbs.cpp)

add_executable(run_app
    app/main.cpp
    src/simulator.cpp
    src/controller.cpp
    ${HEADER_FILES}
)

# Includes
target_include_directories(run_cbs PRIVATE include)
target_include_directories(run_app PRIVATE include)

# Linking
target_link_libraries(run_cbs ${Boost_LIBRARIES} yaml-cpp)
target_link_libraries(run_app ${Boost_LIBRARIES} yaml-cpp sfml-system sfml-window sfml-graphics ${CMAKE_THREAD_LIBS_INIT})
