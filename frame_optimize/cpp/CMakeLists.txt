cmake_minimum_required(VERSION 3.14)
project(beam_fem_standalone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
include(CheckCXXCompilerFlag)

# Build executables
add_executable(beam_fem_standalone beam_fem_standalone.cpp)
add_executable(beam_fem_server beam_fem_server.cpp)

target_link_libraries(beam_fem_standalone
    PRIVATE
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

target_link_libraries(beam_fem_server
    PRIVATE
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
        Threads::Threads
)

function(configure_fem_target target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /O2 /W4)
    else()
        target_compile_options(${target_name} PRIVATE -O3 -Wall -Wextra)
        check_cxx_compiler_flag("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
        if(COMPILER_SUPPORTS_MARCH_NATIVE)
            target_compile_options(${target_name} PRIVATE -march=native)
        endif()
    endif()
endfunction()

configure_fem_target(beam_fem_standalone)
configure_fem_target(beam_fem_server)

if(WIN32)
    target_link_libraries(beam_fem_server PRIVATE ws2_32)
endif()

# Install
install(TARGETS beam_fem_standalone beam_fem_server
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/../bin)

# Closed-form benchmarks (cantilever incl. shear term, tube torsion, ...)
enable_testing()
add_test(NAME beam_fem_closed_form COMMAND beam_fem_standalone --selftest)
