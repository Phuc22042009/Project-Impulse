cmake_minimum_required(VERSION 3.20)

project(PhysicsEngine
    VERSION 1.0.0
    DESCRIPTION "2D Rigid Body Physics Engine with Raylib & ImGui"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

set(CMAKE_WARN_DEPRECATED ON CACHE BOOL "" FORCE)

# ==========================================
# 3. Thu thập file nguồn dự án
# ==========================================
file(GLOB_RECURSE ENGINE_SOURCES CONFIGURE_DEPENDS
    "src/physics/*.cpp"
    "src/math/*.cpp"
)

add_library(PhysicsEngineCore STATIC ${ENGINE_SOURCES})

target_include_directories(PhysicsEngineCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(MSVC)
    target_compile_options(PhysicsEngineCore PRIVATE /W4)
else()
    target_compile_options(PhysicsEngineCore PRIVATE -Wall -Wextra)
endif()

add_executable(PhysicsEngineDemo
    src/main.cpp
)

target_include_directories(PhysicsEngineDemo PRIVATE
    src
)

target_link_libraries(PhysicsEngineDemo PRIVATE
    PhysicsEngineCore
    raylib
)