cmake_minimum_required(VERSION 3.1)

project(cameraCode)
list(APPEND CMAKE_PREFIX_PATH "/opt/MVS/lib/64/")

find_library(CAMERA_LIB MvCameraControl REQUIRED )   

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system filesystem)

find_package(Threads)

find_library(PLCIO_LIBRARY plc)

add_executable(${PROJECT_NAME} cameraCode.cpp)

target_include_directories(${PROJECT_NAME} 
                        PUBLIC
                        ${OpenCV_INCLUDE_DIRS}
                        ${Boost_INCLUDE_DIRS}
                        /opt/MVS/include 
                        /usr/local/include
                        )

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${CAMERA_LIB}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PLCIO_LIBRARY}
    
)

link_directories(${PROJECT_NAME} ${Boost_LIBRARY_DIRS})

