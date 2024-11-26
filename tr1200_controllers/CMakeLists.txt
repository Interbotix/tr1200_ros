cmake_minimum_required(VERSION 3.10.0)
project(tr1200_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  pluginlib
  realtime_tools
  tr1200_msgs
)

foreach(DEP IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(tr1200_rc_state_broadcaster_parameters
  src/tr1200_rc_state_broadcaster_parameters.yaml
)

add_library(
  tr1200_rc_state_broadcaster SHARED
  src/tr1200_rc_state_broadcaster.cpp
)

target_include_directories(tr1200_rc_state_broadcaster
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tr1200_rc_state_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(tr1200_rc_state_broadcaster
  tr1200_rc_state_broadcaster_parameters
)

pluginlib_export_plugin_description_file(
  controller_interface plugin_description.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    tr1200_rc_state_broadcaster
    tr1200_rc_state_broadcaster_parameters
  EXPORT export_tr1200_rc_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_tr1200_rc_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
