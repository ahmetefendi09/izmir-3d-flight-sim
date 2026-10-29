cmake_minimum_required(VERSION 3.16)
project(IzmirFlightSim VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(CURL REQUIRED)
find_package(PNG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external
    ${CURL_INCLUDE_DIRS}
    ${PNG_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
)

add_executable(phase1_verify src/phase1_verify.cpp)

add_executable(izmir_gis_compiler src/gis_compiler.cpp)
target_link_libraries(izmir_gis_compiler PRIVATE ${CURL_LIBRARIES} ${PNG_LIBRARIES})

add_executable(izmir_sim_phase4 src/izmir_sim_main.cpp)
target_link_libraries(izmir_sim_phase4 PRIVATE ${OPENGL_LIBRARIES} ${X11_LIBRARIES} m pthread)
