cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG master
    )

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

file(GLOB_RECURSE ENGINE_SRC "src/*.cpp")

file(GLOB IMGUI_SRC "external/imgui/*.cpp")
set(RLIMGUI_SRC "external/rlImGui/rlImGui.cpp")

add_executable(PhysicsEngine src/main.cpp)
target_include_directories(PhysicsEngine PRIVATE Math)
target_include_directories(PhysicsEngine PRIVATE 
    src
    external/imgui
    external/rlImGui
)
target_link_libraries(PhysicsEngine PRIVATE raylib)