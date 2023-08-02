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
file(GLOB COMMON_FILES src/common/*.cpp src/common/*.hpp)
add_executable(${PROJECT_NAME}_gui ${COMMON_FILES} src/main.cpp)
target_link_libraries(${PROJECT_NAME}_gui chipmunk_static igl::glfw igl::imgui)

add_executable(${PROJECT_NAME}_play ${COMMON_FILES} src/play.cpp)
target_link_libraries(${PROJECT_NAME}_play chipmunk_static igl::glfw igl::imgui)