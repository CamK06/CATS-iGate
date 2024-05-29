cmake_minimum_required(VERSION 3.12)
project(cats-igate)

find_package(PkgConfig REQUIRED)
pkg_check_modules(inih REQUIRED inih)

include(FetchContent)
FetchContent_Declare(
    libCATS
    GIT_REPOSITORY https://github.com/CamK06/libCATS.git
)
FetchContent_Declare(
    UUID
    GIT_REPOSITORY https://github.com/crashoz/uuid_v4.git
)
FetchContent_Declare(
    flog
    GIT_REPOSITORY https://github.com/AlexandreRouma/flog.git
)
FetchContent_MakeAvailable(flog libCATS UUID)

add_subdirectory(proto)

file(GLOB SOURCES src/*.cpp ${flog_SOURCE_DIR}/lib/flog.cpp)

include_directories(cats-igate ${CMAKE_CURRENT_BINARY_DIR} ${flog_SOURCE_DIR}/lib ${INIReader_INCLUDE_DIRS} ${uuid_SOURCE_DIR})
add_executable(cats-igate ${SOURCES})
target_link_libraries(cats-igate proto CATS INIReader)