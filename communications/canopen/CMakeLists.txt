find_path(MOTOR_MANAGER_LELY_INCLUDE_DIR lely/io2/linux/can.hpp
  HINTS /usr/local/include /usr/include /home/mini0/lely-core/include
  DOC "Lely CAN I/O headers")
if(NOT MOTOR_MANAGER_LELY_INCLUDE_DIR)
  message(FATAL_ERROR
    "Lely CAN I/O headers not found. Install lely-core or set MOTOR_MANAGER_LELY_INCLUDE_DIR.")
endif()

find_library(MOTOR_MANAGER_LELY_IO2_LIB lely-io2
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /home/mini0/lely-core/src/io2/.libs)
find_library(MOTOR_MANAGER_LELY_EV_LIB lely-ev
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /home/mini0/lely-core/src/ev/.libs)
find_library(MOTOR_MANAGER_LELY_CAN_LIB lely-can
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /home/mini0/lely-core/src/can/.libs)
find_library(MOTOR_MANAGER_LELY_UTIL_LIB lely-util
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /home/mini0/lely-core/src/util/.libs)
find_library(MOTOR_MANAGER_LELY_LIBC_LIB lely-libc
  HINTS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu /home/mini0/lely-core/src/libc/.libs)

if(NOT MOTOR_MANAGER_LELY_IO2_LIB OR
   NOT MOTOR_MANAGER_LELY_EV_LIB OR
   NOT MOTOR_MANAGER_LELY_CAN_LIB OR
   NOT MOTOR_MANAGER_LELY_UTIL_LIB OR
   NOT MOTOR_MANAGER_LELY_LIBC_LIB)
  message(FATAL_ERROR
    "Lely CAN I/O libraries not found. Build/install lely-core before enabling canopen.")
endif()

add_library(canopen
  src/canopen_master.cpp
  src/canopen_controller.cpp
)

target_include_directories(canopen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${MOTOR_MANAGER_LELY_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(canopen
  PUBLIC
    motor_interface::motor_interface
    "${MOTOR_MANAGER_LELY_IO2_LIB}"
    "${MOTOR_MANAGER_LELY_EV_LIB}"
    "${MOTOR_MANAGER_LELY_CAN_LIB}"
    "${MOTOR_MANAGER_LELY_UTIL_LIB}"
    "${MOTOR_MANAGER_LELY_LIBC_LIB}"
)

target_compile_features(canopen PUBLIC cxx_std_17)

add_library(canopen::canopen ALIAS canopen)
