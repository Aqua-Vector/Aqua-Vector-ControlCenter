# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(TorpedoControlSystem)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
include_directories(include/torpedo)
include_directories(src)

# 1. src 안의 모든 cpp 파일을 찾음
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp" "src/*.c")

# 2. 메인 함수가 들어있는 두 파일을 소스 목록에서 제외 (공통 라이브러리만 남김)
list(REMOVE_ITEM ALL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
list(REMOVE_ITEM ALL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main_lidar_algo_test.cpp")

# --- [빌드 1] 기존 전체 통제소 시스템 ---
add_executable(torpedo_control src/main.cpp ${ALL_SOURCES})
target_link_libraries(torpedo_control pthread rt)

# --- [빌드 2] 라이다 알고리즘 단독 테스트 ---
add_executable(lidar_test src/main_lidar_algo_test.cpp ${ALL_SOURCES})
target_link_libraries(lidar_test pthread rt)