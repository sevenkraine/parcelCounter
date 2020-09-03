cmake_minimum_required(VERSION 3.1)

project(cameraCode)
list(APPEND CMAKE_PREFIX_PATH "/opt/MVS/lib/64/")

find_library(CAMERA_LIB MvCameraControl REQUIRED )   

find_package(OpenCV REQUIRED)

find_package(Threads)

add_executable(${PROJECT_NAME} cameraCode.cpp)

target_include_directories(${PROJECT_NAME} 
                        PUBLIC
                        ${OpenCV_INCLUDE_DIRS}
                        /opt/MVS/include 
                        )

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${CAMERA_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
)

