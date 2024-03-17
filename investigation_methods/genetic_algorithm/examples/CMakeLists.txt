cmake_minimum_required(VERSION 3.5)
project(genetic_algorithm_example)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set optimisation level
set(CMAKE_CXX_FLAGS "-O3")

# Enable all warnings and treat them as errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add common library
add_library(common STATIC ../../common/pid_controller.cpp)
target_include_directories(common PUBLIC ../../common)

# Add the executable
add_executable(main main.cpp)
target_link_libraries(main PRIVATE common)

# Install targets
install(TARGETS main DESTINATION bin)
install(TARGETS common DESTINATION lib)
