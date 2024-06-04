cmake_minimum_required(VERSION 2.8.9)
PROJECT(robot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include/)
include_directories(/usr/include/ )

# link directories
link_directories("/usr/local/lib")
link_directories("/usr/lib")

#identify the processor, mark "DYNAMIXELLIB"
message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

file(GLOB MAIN
     "./src/*.cpp"
)

# ADD_EXECUTABLE(main ${MAIN} ${MOTIONCONTROL_SRC} )
# target_link_libraries(main /usr/local/lib/libqpOASES.a)

ADD_EXECUTABLE(main ${MAIN}) 
target_link_libraries(main)

