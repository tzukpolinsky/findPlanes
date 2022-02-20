cmake_minimum_required(VERSION 3.21)
project(findPlanes)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
find_package(OpenCV 3.0 QUIET)

find_package(Pangolin REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${Pangolin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/Point.cpp
        src/Navigation.cpp
        src/Auxiliary.cpp
        src/Line.cpp)


target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        -lpthread ${PYTHON_LIBRARIES}
        )
set(CMAKE_CXX_STANDARD 17)

add_executable(test main.cpp)
target_link_libraries(test ${PROJECT_NAME} cnpy)

add_executable(carDataset carDataset.cpp)
target_link_libraries(carDataset ${PROJECT_NAME} cnpy)