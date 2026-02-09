cmake_minimum_required(VERSION 3.5)
project(motor_manager)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -march=native -DNDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g)
endif()

find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ETHERCAT libethercat)

include_directories(include ${ETHERCAT_INCLUDE_DIRS})

set(MOTOR_MANAGER_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motor_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ethercat_master.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ethercat_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/minas_driver.cpp)

add_library(motor_manager_library SHARED ${MOTOR_MANAGER_SOURCES})

target_link_libraries(motor_manager_library PRIVATE ${ETHERCAT_LIBRARIES} yaml-cpp)

install(TARGETS motor_manager_library
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

set(MOTOR_MANAGER_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test.cpp)

add_executable(motor_manager_test ${MOTOR_MANAGER_TEST_SOURCES})

target_link_libraries(motor_manager_test motor_manager_library)