# Config for motor_manager
# Install layout: share/motor_manager/cmake/ -> package prefix is ../../..
get_filename_component(motor_manager_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(motor_manager_PREFIX "${motor_manager_DIR}/../../.." ABSOLUTE)
set(motor_manager_INCLUDE_DIRS "${motor_manager_PREFIX}/include")
set(motor_manager_LIBRARY_DIR "${motor_manager_PREFIX}/lib")

# Imported target for linking
if(NOT TARGET motor_manager_library)
    add_library(motor_manager_library SHARED IMPORTED)
    set_target_properties(motor_manager_library PROPERTIES
        IMPORTED_LOCATION "${motor_manager_LIBRARY_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}motor_manager_library${CMAKE_SHARED_LIBRARY_SUFFIX}"
        INTERFACE_INCLUDE_DIRECTORIES "${motor_manager_INCLUDE_DIRS}"
    )
endif()
