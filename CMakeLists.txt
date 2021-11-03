cmake_minimum_required(VERSION 3.16)
project(robotic-mm-optimization)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(SRC_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/")
set(DEPENDENCIES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/")


###################################
######## configure PROJECT ########
###################################

# set c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wextra -Wuninitialized -pedantic-errors -Wno-unused-parameter)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wc++20-extensions)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)


########################################
######## configure DEPENDENCIES ########
########################################

# libigl settings
# make compilation faster using static linking
option(LIBIGL_USE_STATIC_LIBRARY 	 "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       OFF)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            ON)

# Include libigl
set(LIBIGL_DIR "${DEPENDENCIES_DIR}/libigl/")
set(LIBIGL_INCLUDE_DIR "${LIBIGL_DIR}/include/")
list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)


# Include cppOpt
include_directories("${DEPENDENCIES_DIR}/cppOpt/inc")


# Include & link Ipopt lib

# message("before set IPOPT_DIR = ${IPOPT_DIR}")
# set(IPOPT_DIR CACHE PATH "Ipopt top-level directory")
# message("after set IPOPT_DIR = ${IPOPT_DIR}")

# if(NOT ${IPOPT_DIR})
#     message("IPOPT_DIR is not set")
# endif()

if(WIN32)
    include_directories("${DEPENDENCIES_DIR}/Ipopt/include/coin-or")

	set(IPOPT_DIR "${DEPENDENCIES_DIR}/Ipopt/libWIN" CACHE PATH "directory where Ipopt lib is found")
    link_directories("${IPOPT_DIR}/lib")
    link_libraries("ipopt.dll.lib")


    # find_library(IPOPT_LIB 
    #     "ipopt" 
    #     PATHS "${IPOPT_DIR}" "${IPOPT_DIR}/lib" "${DEPENDENCIES_DIR}/Ipopt/libWIN/lib"
    #     )
    # message("after find_library IPOPT_LIB = ${IPOPT_LIB}")

    # if (${IPOPT_LIB} MATCHES "NOTFOUND")
    #     message("ERROR: Could not find Ipopt lib for Windows.")
    # else()
    #     get_filename_component(IPOPT_LIB_FILE "${IPOPT_LIB}" NAME)
    #     link_libraries(${IPOPT_LIB_FILE})
    #     message("Found Ipopt library for Windows named ${IPOPT_LIB_FILE}")
        
    #     get_filename_component(IPOPT_LIB_PATH "${IPOPT_LIB}" DIRECTORY)
    #     link_directories(${IPOPT_LIB_PATH})
    #     message("Found Ipopt library for Windows at ${IPOPT_LIB_PATH}")
    #     set(IPOPT_DIR "${IPOPT_LIB_PATH}/../")
    #     message("IPOPT_DIR = ${IPOPT_DIR}")
        
    #     find_file(IPOPT_INC "*.h*" PATHS "${IPOPT_DIR}" "${DEPENDENCIES_DIR}/Ipopt/include/coin-or")
    #     message("IPOPT_INC = ${IPOPT_INC}")

    #     include_directories("${DEPENDENCIES_DIR}/Ipopt/include/coin-or")
    # endif()
elseif(APPLE)
    message("TODO implement finding Ipopt on MACOSX")
    message("Please run cmake again and specify the top-level directory of your Ipopt impmenentation:\n
            'make ../ -DIPOPT_DIR:PATH=<path/to/your/Ipopt>'")
endif()


#################################
######## include SOURCES ########
#################################

include_directories(${SRC_ROOT_PATH})

# Add header files recursively
file(
    GLOB_RECURSE HEADER_FILES 
    LIST_DIRECTORIES false
    "${SRC_ROOT_PATH}/*.h*"
)

# Add source files recursively
file(
    GLOB_RECURSE SRC_FILES 
    LIST_DIRECTORIES false
    "${SRC_ROOT_PATH}/*.c*"
	"${SRC_ROOT_PATH}/*.h*"
)


# # Add source files recursively
# file(
    # GLOB_RECURSE COMMON_SRC_FILES 
    # LIST_DIRECTORIES false
    # "${SRC_ROOT_PATH}/*.c*"
	# "${SRC_ROOT_PATH}/*.h*"
# )

# file(
    # GLOB EXECUTABLE_SRC_FILES
    # LIST_DIRECTORIES false
    # "${SRC_ROOT_PATH}/bin/*.c*"
# )
# message(${COMMON_SRC_FILES})
# message(${EXECUTABLE_SRC_FILES})

# list(REMOVE_ITEM COMMON_SRC_FILES ${EXECUTABLE_SRC_FILES})

# message(${COMMON_SRC_FILES})
# message(${EXECUTABLE_SRC_FILES})


# Group source files for IDE (Visual Studio) -- remove this if it causes problems for other IDEs
foreach(SRC_FILE IN ITEMS ${SRC_FILES})
    get_filename_component(SRC_FILE_PATH "${SRC_FILE}" PATH)
    file(RELATIVE_PATH SRC_PATH_RELATIVE "${SRC_ROOT_PATH}" "${SRC_FILE_PATH}")
    string(REPLACE "/" "\\" GROUP_PATH "${SRC_PATH_RELATIVE}")
    source_group("${GROUP_PATH}" FILES "${SRC_FILE}")
endforeach()

if(WIN32)
	add_definitions (-D_USE_MATH_DEFINES)
	message("  use -D_USE_MATH_DEFINES on windows")
endif()



link_libraries(igl::core igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui igl::xml)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    link_libraries(stdc++fs)
endif()

add_executable(${PROJECT_NAME} ${SRC_FILES})

# WIN: post-build event to copy Ipopt dlls to target directory 
if(WIN32)
message("post build IPOPT_DIR = ${IPOPT_DIR}")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${IPOPT_DIR}/bin"
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif()


# # Add your project files
# file(GLOB SRC_FILES *.cpp)
# add_executable(${PROJECT_NAME} ${SRC_FILES})
# target_link_libraries(${PROJECT_NAME} PUBLIC igl::core igl::opengl_glfw)

# target_compile_features(example PRIVATE cxx_std_17)