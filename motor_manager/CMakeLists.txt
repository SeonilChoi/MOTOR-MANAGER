add_library(motor_manager SHARED src/motor_manager.cpp)

# IgH EtherCAT is commonly installed below /opt/etherlab, which is not in the
# dynamic linker's default search path.  Preserve the directory found at
# configure time so installed ROS 2 executables can load libethercat.so.
get_filename_component(MOTOR_MANAGER_IGH_ETHERCAT_LIB_DIR
  "${MOTOR_MANAGER_IGH_ETHERCAT_LIB}" DIRECTORY)
set_target_properties(motor_manager PROPERTIES
  BUILD_RPATH "${MOTOR_MANAGER_IGH_ETHERCAT_LIB_DIR}"
  INSTALL_RPATH "${MOTOR_MANAGER_IGH_ETHERCAT_LIB_DIR}"
)

target_include_directories(motor_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(motor_manager
  PUBLIC  motor_interface::motor_interface
  PUBLIC  "${MOTOR_MANAGER_IGH_ETHERCAT_LIB}"
  PRIVATE ethercat::ethercat
  PRIVATE canopen::canopen
  PRIVATE serial::serial
  PRIVATE socketcan::socketcan
  PRIVATE minas::minas
  PRIVATE zeroerr::zeroerr
  PRIVATE dynamixel::dynamixel
  PRIVATE cubemars::cubemars
  PRIVATE ament_index_cpp::ament_index_cpp
  PRIVATE yaml-cpp
)

target_compile_features(motor_manager PUBLIC cxx_std_17)

add_library(motor_manager::motor_manager ALIAS motor_manager)
