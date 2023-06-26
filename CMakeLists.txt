cmake_minimum_required(VERSION 3.16)
project(dynamic_mm)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)
add_subdirectory(external/Chipmunk2D)
include_directories(external/Chipmunk2D/include)

# Enable the target igl::glfw
igl_include(glfw)
igl_include(imgui)

# Add your project files
file(GLOB SRC_FILES src/*.cpp src/*.hpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} chipmunk_static igl::glfw igl::imgui)
