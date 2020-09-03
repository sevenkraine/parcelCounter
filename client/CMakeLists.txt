cmake_minimum_required(VERSION 3.5)

project(client)

set(CMAKE_PREFIX_PATH /home/wolf/projects/server/builddir/client/install /home/wolf/projects/server/builddir /home/wolf/projects/server/builddir/curl/install)

find_package(OpenCV REQUIRED)

find_package(Threads REQUIRED)

find_package(RapidJSON CONFIG REQUIRED)

find_package(TRITON CONFIG REQUIRED)

# find_package(Triton)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rapidjson)

add_library(
      json-utils-library EXCLUDE_FROM_ALL OBJECT
      json-utils-library/json_utils.h json-utils-library/json_utils.cc
      json-utils-library/common.h json-utils-library/common.cc
)

add_executable(${PROJECT_NAME} client.cpp)


# find_library(rapidJSON ${CMAKE_CURRENT_SOURCE_DIRECTORY}/client/server/clients/lib/grpcclient SHARED)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    json-utils-library
    TRITON::grpcclient_static
    TRITON::httpclient_static
    # TRITON::common
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
)

target_include_directories(${PROJECT_NAME}
                            PUBLIC
                            ${OpenCV_INCLUDE_DIRS}
                            ${RapidJSON_INCLUDE_DIRS}
                            json-utils-library
                            )   

