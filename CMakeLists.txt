cmake_minimum_required(VERSION 3.16)
project(hakoniwa_sensor_adapter_airsim LANGUAGES CXX)

set(HAKONIWA_SENSOR_ADAPTER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-sensor-adapter"
    CACHE PATH "Path to the publicly available hakoniwa-sensor-adapter checkout")
if(NOT EXISTS "${HAKONIWA_SENSOR_ADAPTER_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "Clone hakoniwa-sensor-adapter beside this repository or set HAKONIWA_SENSOR_ADAPTER_ROOT")
endif()
add_subdirectory("${HAKONIWA_SENSOR_ADAPTER_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/sensor-adapter")
find_package(Boost REQUIRED)
add_library(hakoniwa_sensor_adapter_airsim STATIC
    src/projectairsim_imu_adapter.cpp
    src/projectairsim_gps_adapter.cpp)
target_compile_features(hakoniwa_sensor_adapter_airsim PUBLIC cxx_std_17)
target_include_directories(hakoniwa_sensor_adapter_airsim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(hakoniwa_sensor_adapter_airsim PUBLIC hakoniwa_sensor_adapter)

option(HAKONIWA_SENSOR_ADAPTER_AIRSIM_BUILD_TESTS "Build standalone compatibility tests" OFF)
if(HAKONIWA_SENSOR_ADAPTER_AIRSIM_BUILD_TESTS)
    enable_testing()
    add_executable(projectairsim_imu_adapter_test test/projectairsim_imu_adapter_test.cpp)
    target_link_libraries(projectairsim_imu_adapter_test PRIVATE hakoniwa_sensor_adapter_airsim)
    target_compile_definitions(projectairsim_imu_adapter_test PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME projectairsim_imu_adapter COMMAND projectairsim_imu_adapter_test)

    add_executable(projectairsim_gps_adapter_test test/projectairsim_gps_adapter_test.cpp)
    target_link_libraries(projectairsim_gps_adapter_test PRIVATE hakoniwa_sensor_adapter_airsim)
    target_compile_definitions(projectairsim_gps_adapter_test PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME projectairsim_gps_adapter COMMAND projectairsim_gps_adapter_test)
endif()
