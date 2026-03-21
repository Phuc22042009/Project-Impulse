cmake_minimum_required(VERSION 3.15)

project(PhysicsEngine)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG master
    )


FetchContent_MakeAvailable(raylib)

add_executable(PhysicsEngine src/main.cpp)
target_include_directories(PhysicsEngine PRIVATE Math)
target_include_directories(PhysicsEngine PRIVATE Physics)
target_link_libraries(PhysicsEngine PRIVATE raylib)