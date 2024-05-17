cmake_minimum_required(VERSION 3.12)
project(cats-igate)

include(FetchContent)
FetchContent_Declare(
    libCATS
    GIT_REPOSITORY https://github.com/CamK06/libCATS.git
)
FetchContent_MakeAvailable(libCATS)

FetchContent_Declare(
    UUIDV4
    GIT_REPOSITORY https://github.com/crashoz/uuid_v4.git
)
FetchContent_MakeAvailable(UUIDV4)

FetchContent_Declare(
    flog
    GIT_REPOSITORY https://github.com/AlexandreRouma/flog.git
)
FetchContent_MakeAvailable(flog)

add_subdirectory(proto)

set(OPT_BUILD_RADIO_IFACE ON)

file(GLOB SOURCES src/*.cpp ${flog_SOURCE_DIR}/lib/flog.cpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${flog_SOURCE_DIR}/lib)
add_executable(cats-igate ${SOURCES})
target_link_libraries(cats-igate proto CATS)

