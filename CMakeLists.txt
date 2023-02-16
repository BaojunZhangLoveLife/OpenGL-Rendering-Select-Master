#set cmake version number
cmake_minimum_required(VERSION 3.4.0)

#set project name
project(OpenGL-Rendering-Select-Master)
SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_INCLUDE_CURRENT_DIR ON) 
# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Open3D
find_package(Open3D)
include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})
add_definitions(${Open3D_DEFINITIONS})

# Surface
if(NOT SURFACE_ROOT)
        set(SURFACE_ROOT "" CACHE PATH "surface root path")
endif()
if(SURFACE_ROOT)
	set(SURFACE_LIB_DIR ${SURFACE_ROOT}/lib/)
    set(SURFACE_BIN_DIR ${SURFACE_ROOT}/bin/)
    file(GLOB SURFACE_LIB_FILES ${SURFACE_ROOT}/lib/*.lib)
    file(GLOB SURFACE_BIN_FILES ${SURFACE_ROOT}/bin/*.dll)
    set(SURFACE_INC_DIR ${SURFACE_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without SURFACE")
endif(SURFACE_ROOT)
include_directories(${SURFACE_INC_DIR})

# Glut
if(NOT GLUT_ROOT)
        set(GLUT_ROOT "" CACHE PATH "surface glut root")
endif()
if(GLUT_ROOT)
	set(GLUT_LIB_DIR ${GLUT_ROOT}/lib/)
    set(GLUT_BIN_DIR ${GLUT_ROOT}/bin/)
    file(GLOB GLUT_LIB_FILES ${GLUT_ROOT}/lib/*.lib)
    file(GLOB GLUT_BIN_FILES ${GLUT_ROOT}/bin/*.dll)
    set(GLUT_INC_DIR ${GLUT_ROOT}/include/)
else()
    message(FATAL_ERROR "can not generate without GLUT")
endif(GLUT_ROOT)
include_directories(${GLUT_INC_DIR})

# pcl
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_AUTOMOC ON) 
# qt
find_package(Qt5Widgets)  
find_package(Qt5Core)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5OpenGLExtensions)
file(GLOB SRC_FILES "./*.cpp" "./*.cxx" "./*.c") 
file(GLOB HEAD_FILES "./*.h" "./*.hpp" "./*.hxx") 
file(GLOB UI_FILES "./UI/*.ui") 
file(GLOB SHADER_FILES "./Shader/*.frag" "./Shader/*.vert") 

qt5_wrap_ui(WRAP_FILES ${UI_FILES})
qt5_add_resources(QRC_FILES ${RCC_FILES})

source_group("Ui" FILES ${UI_FILES})
source_group("Source" FILES ${SRC_FILES})
source_group("Header" FILES ${HEAD_FILES})
source_group("Shader" FILES ${SHADER_FILES})

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEAD_FILES} ${QRC_FILES} ${WRAP_FILES} ${SHADER_FILES})

target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL Qt5::OpenGLExtensions)
target_link_libraries(${PROJECT_NAME} ${Open3D_LIBRARIES})
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries (${PROJECT_NAME} ${SURFACE_LIB_FILES})
target_link_libraries (${PROJECT_NAME} ${GLUT_LIB_FILES})

#copy file from source dirtory to binaray dirtory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${PROJECT_BINARY_DIR}/Release)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Shader DESTINATION ${PROJECT_BINARY_DIR}/Release)

# generate libraies
#add_library(DataProcessing DataProcessing.cpp)
#target_link_libraries(DataProcessing Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL)
