cmake_minimum_required(VERSION 3.19)
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
    src/projectairsim_gps_adapter.cpp
    src/projectairsim_baro_adapter.cpp
    src/projectairsim_mag_adapter.cpp)
target_compile_features(hakoniwa_sensor_adapter_airsim PUBLIC cxx_std_17)
target_include_directories(hakoniwa_sensor_adapter_airsim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(hakoniwa_sensor_adapter_airsim PUBLIC hakoniwa_sensor_adapter)

# Native adapters: Project AirSim's own core_sim sensors, compiled unchanged from the files pinned in
# third_party/projectairsim/lock.json. The files are not in this repository; fetch them first with
# `python tools/projectairsim_lock.py fetch` (they go to .cache/projectairsim/<commit>/).
option(HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE "Build the adapters running Project AirSim's core_sim sensors" OFF)
if(HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/third_party/projectairsim/lock.json" PAS_LOCK)
    string(JSON PAS_COMMIT GET "${PAS_LOCK}" commit)
    set(PAS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.cache/projectairsim/${PAS_COMMIT}")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/tools/projectairsim_lock.py" verify
        RESULT_VARIABLE PAS_VERIFY OUTPUT_VARIABLE PAS_VERIFY_OUT ERROR_VARIABLE PAS_VERIFY_OUT)
    if(NOT PAS_VERIFY EQUAL 0)
        message(FATAL_ERROR "Project AirSim sources at ${PAS_COMMIT} are missing or changed in the cache:\n"
            "${PAS_VERIFY_OUT}Run: python tools/projectairsim_lock.py fetch")
    endif()
    string(JSON PAS_SOURCE_COUNT LENGTH "${PAS_LOCK}" sources)
    math(EXPR PAS_LAST "${PAS_SOURCE_COUNT} - 1")
    set(PAS_SOURCES "")
    foreach(i RANGE ${PAS_LAST})
        string(JSON source GET "${PAS_LOCK}" sources ${i})
        list(APPEND PAS_SOURCES "${PAS_ROOT}/${source}")
    endforeach()

    # core_sim's third-party libraries (as core_sim builds them: nng 1.x, msgpack without Boost).
    find_package(Eigen3 REQUIRED NO_MODULE)
    find_path(PAS_JSON_DIR json.hpp PATH_SUFFIXES nlohmann REQUIRED)
    find_path(PAS_MSGPACK_DIR msgpack.hpp REQUIRED)
    find_path(PAS_NNG_DIR nng/nng.h REQUIRED)
    find_library(PAS_NNG_LIBRARY nng REQUIRED)
    file(STRINGS "${PAS_NNG_DIR}/nng/nng.h" PAS_NNG_MAJOR REGEX "#define NNG_MAJOR_VERSION")
    if(NOT PAS_NNG_MAJOR MATCHES "NNG_MAJOR_VERSION 1$")
        message(FATAL_ERROR "core_sim needs nng 1.x (it pins v1.11); found: ${PAS_NNG_MAJOR}")
    endif()

    add_library(projectairsim_core_sim_subset STATIC ${PAS_SOURCES} src/native/core_sim_stubs.cpp)
    target_include_directories(projectairsim_core_sim_subset
        PUBLIC "${PAS_ROOT}/core_sim/include" "${PAS_ROOT}/core_sim/src" ${PAS_JSON_DIR} ${PAS_MSGPACK_DIR} ${PAS_NNG_DIR})
    target_compile_definitions(projectairsim_core_sim_subset PUBLIC MSGPACK_NO_BOOST)
    target_compile_features(projectairsim_core_sim_subset PUBLIC cxx_std_17)
    # Upstream code is compiled as it is; its warnings are not ours to fix.
    if(NOT MSVC)
        target_compile_options(projectairsim_core_sim_subset PRIVATE -w)
    endif()
    target_link_libraries(projectairsim_core_sim_subset PUBLIC Eigen3::Eigen ${PAS_NNG_LIBRARY})

    add_library(hakoniwa_sensor_adapter_airsim_native STATIC
        src/native/projectairsim_native_imu_adapter.cpp
        src/native/projectairsim_native_mag_adapter.cpp
        src/native/projectairsim_native_baro_adapter.cpp
        src/native/projectairsim_native_gps_adapter.cpp)
    target_include_directories(hakoniwa_sensor_adapter_airsim_native PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(hakoniwa_sensor_adapter_airsim_native
        PUBLIC hakoniwa_sensor_adapter PRIVATE projectairsim_core_sim_subset)
endif()

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

    add_executable(projectairsim_baro_adapter_test test/projectairsim_baro_adapter_test.cpp)
    target_link_libraries(projectairsim_baro_adapter_test PRIVATE hakoniwa_sensor_adapter_airsim)
    target_compile_definitions(projectairsim_baro_adapter_test
        PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME projectairsim_baro_adapter COMMAND projectairsim_baro_adapter_test)

    add_executable(projectairsim_mag_adapter_test test/projectairsim_mag_adapter_test.cpp)
    target_link_libraries(projectairsim_mag_adapter_test PRIVATE hakoniwa_sensor_adapter_airsim)
    target_compile_definitions(projectairsim_mag_adapter_test
        PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME projectairsim_mag_adapter COMMAND projectairsim_mag_adapter_test)

    if(HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE)
        add_executable(projectairsim_native_imu_adapter_test test/projectairsim_native_imu_adapter_test.cpp)
        target_link_libraries(projectairsim_native_imu_adapter_test
            PRIVATE hakoniwa_sensor_adapter_airsim_native hakoniwa_sensor_adapter_airsim)
        target_compile_definitions(projectairsim_native_imu_adapter_test
            PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
        add_test(NAME projectairsim_native_imu_adapter COMMAND projectairsim_native_imu_adapter_test)
        add_executable(projectairsim_native_mag_baro_gps_adapter_test test/projectairsim_native_mag_baro_gps_adapter_test.cpp)
        target_link_libraries(projectairsim_native_mag_baro_gps_adapter_test
            PRIVATE hakoniwa_sensor_adapter_airsim_native hakoniwa_sensor_adapter_airsim)
        target_compile_definitions(projectairsim_native_mag_baro_gps_adapter_test
            PRIVATE HAKO_AIRSIM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
        add_test(NAME projectairsim_native_mag_baro_gps_adapter COMMAND projectairsim_native_mag_baro_gps_adapter_test)
    endif()
endif()
