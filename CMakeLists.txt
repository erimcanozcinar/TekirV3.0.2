cmake_minimum_required(VERSION 3.10)
project(TekirV3_0_1_Sim LANGUAGES CXX)
set(CMAKE_PREFIX_PATH "/home/erim/raisim_ws_v1.1.7/install")

set(ALLEGRO_INCLUDE_DIR "/usr/local/include/allegro5")
set(ALLEGRO_LIB_DIR "/usr/local/lib")

include_directories(${ALLEGRO_INCLUDE_DIR})

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories (${EIGEN3_INCLUDE_DIRS})
include_directories (${QP_INCLUDE_DIRS})

add_executable(tekirV3 src/main.cpp)

target_link_libraries(tekirV3 PUBLIC raisim::raisim pthread)
target_include_directories(tekirV3 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link the executable to CGAL and third-party libraries
target_link_libraries(tekirV3 PRIVATE "${ALLEGRO_LIB_DIR}/liballegro.so" "${ALLEGRO_LIB_DIR}/liballegro_primitives.so" "${ALLEGRO_LIB_DIR}/liballegro_dialog.so")
# link_directories(/usr/local/lib)
