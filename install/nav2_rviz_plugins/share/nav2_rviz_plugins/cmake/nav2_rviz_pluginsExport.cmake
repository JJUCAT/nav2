# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget nav2_rviz_plugins::nav2_rviz_plugins)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target nav2_rviz_plugins::nav2_rviz_plugins
add_library(nav2_rviz_plugins::nav2_rviz_plugins SHARED IMPORTED)

set_target_properties(nav2_rviz_plugins::nav2_rviz_plugins PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
  INTERFACE_INCLUDE_DIRECTORIES "/ws/src/nav2/install/nav2_lifecycle_manager/include;/ws/src/nav2/install/nav2_util/include;/ws/src/nav2/install/nav2_msgs/include;/opt/ros/foxy/include;/usr/include/x86_64-linux-gnu/qt5/;/usr/include/x86_64-linux-gnu/qt5/QtWidgets;/usr/include/x86_64-linux-gnu/qt5/QtGui;/usr/include/x86_64-linux-gnu/qt5/QtCore;/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++;/opt/ros/foxy/opt/rviz_ogre_vendor/include/OGRE;${_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "geometry_msgs::geometry_msgs__rosidl_generator_c;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c;geometry_msgs::geometry_msgs__rosidl_typesupport_c;geometry_msgs::geometry_msgs__rosidl_generator_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_cpp;nav2_msgs::nav2_msgs__rosidl_generator_c;nav2_msgs::nav2_msgs__rosidl_typesupport_introspection_c;nav2_msgs::nav2_msgs__rosidl_typesupport_c;nav2_msgs::nav2_msgs__rosidl_generator_cpp;nav2_msgs::nav2_msgs__rosidl_typesupport_introspection_cpp;nav2_msgs::nav2_msgs__rosidl_typesupport_cpp;nav_msgs::nav_msgs__rosidl_generator_c;nav_msgs::nav_msgs__rosidl_typesupport_introspection_c;nav_msgs::nav_msgs__rosidl_typesupport_c;nav_msgs::nav_msgs__rosidl_generator_cpp;nav_msgs::nav_msgs__rosidl_typesupport_introspection_cpp;nav_msgs::nav_msgs__rosidl_typesupport_cpp;pluginlib::pluginlib;rclcpp::rclcpp;rclcpp_lifecycle::rclcpp_lifecycle;rviz_common::rviz_common;rviz_default_plugins::rviz_default_plugins;rviz_rendering::rviz_rendering;std_msgs::std_msgs__rosidl_generator_c;std_msgs::std_msgs__rosidl_typesupport_introspection_c;std_msgs::std_msgs__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_generator_cpp;std_msgs::std_msgs__rosidl_typesupport_introspection_cpp;std_msgs::std_msgs__rosidl_typesupport_cpp;tf2_geometry_msgs::tf2_geometry_msgs;/ws/src/nav2/install/nav2_lifecycle_manager/lib/libnav2_lifecycle_manager_core.so;/ws/src/nav2/install/nav2_util/lib/libnav2_util_core.so;/ws/src/nav2/install/nav2_msgs/lib/libnav2_msgs__rosidl_generator_c.so;/ws/src/nav2/install/nav2_msgs/lib/libnav2_msgs__rosidl_typesupport_introspection_c.so;nav2_msgs::nav2_msgs__rosidl_generator_c;/ws/src/nav2/install/nav2_msgs/lib/libnav2_msgs__rosidl_typesupport_c.so;/ws/src/nav2/install/nav2_msgs/lib/libnav2_msgs__rosidl_typesupport_introspection_cpp.so;/ws/src/nav2/install/nav2_msgs/lib/libnav2_msgs__rosidl_typesupport_cpp.so;nav_msgs::nav_msgs__rosidl_typesupport_introspection_c;nav_msgs::nav_msgs__rosidl_typesupport_c;nav_msgs::nav_msgs__rosidl_generator_cpp;nav_msgs::nav_msgs__rosidl_typesupport_introspection_cpp;nav_msgs::nav_msgs__rosidl_typesupport_cpp;/opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so;/opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so;/opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libcomposition_interfaces__rosidl_generator_c.so;/opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libcomponent_manager.so;ament_index_cpp::ament_index_cpp;class_loader::class_loader;composition_interfaces::composition_interfaces__rosidl_generator_c;composition_interfaces::composition_interfaces__rosidl_typesupport_introspection_c;composition_interfaces::composition_interfaces__rosidl_typesupport_c;composition_interfaces::composition_interfaces__rosidl_generator_cpp;composition_interfaces::composition_interfaces__rosidl_typesupport_introspection_cpp;composition_interfaces::composition_interfaces__rosidl_typesupport_cpp;/opt/ros/foxy/lib/liborocos-kdl.so.1.4.0;/opt/ros/foxy/lib/libtf2.so;console_bridge::console_bridge;/opt/ros/foxy/lib/libtf2_ros.so;message_filters::message_filters;rclcpp_action::rclcpp_action;/opt/ros/foxy/lib/libstatic_transform_broadcaster_node.so;rclcpp_components::component_manager;tf2_msgs::tf2_msgs__rosidl_generator_c;tf2_msgs::tf2_msgs__rosidl_typesupport_introspection_c;tf2_msgs::tf2_msgs__rosidl_typesupport_c;tf2_msgs::tf2_msgs__rosidl_generator_cpp;tf2_msgs::tf2_msgs__rosidl_typesupport_introspection_cpp;tf2_msgs::tf2_msgs__rosidl_typesupport_cpp;/opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libnav_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_introspection_c.so;nav_msgs::nav_msgs__rosidl_generator_c;/opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_cpp.so;std_msgs::std_msgs__rosidl_typesupport_introspection_c;std_msgs::std_msgs__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_generator_cpp;std_msgs::std_msgs__rosidl_typesupport_introspection_cpp;std_msgs::std_msgs__rosidl_typesupport_cpp;/opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so;/opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libaction_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libaction_msgs__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/libtest_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libtest_msgs__rosidl_typesupport_introspection_c.so;test_msgs::test_msgs__rosidl_generator_c;/opt/ros/foxy/lib/libtest_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libtest_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libtest_msgs__rosidl_typesupport_cpp.so;unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_c;unique_identifier_msgs::unique_identifier_msgs__rosidl_typesupport_introspection_c;unique_identifier_msgs::unique_identifier_msgs__rosidl_typesupport_c;unique_identifier_msgs::unique_identifier_msgs__rosidl_generator_cpp;unique_identifier_msgs::unique_identifier_msgs__rosidl_typesupport_introspection_cpp;unique_identifier_msgs::unique_identifier_msgs__rosidl_typesupport_cpp;/opt/ros/foxy/lib/librcl.so;rcl_interfaces::rcl_interfaces__rosidl_generator_c;rcl_interfaces::rcl_interfaces__rosidl_typesupport_introspection_c;rcl_interfaces::rcl_interfaces__rosidl_typesupport_c;rcl_interfaces::rcl_interfaces__rosidl_generator_cpp;rcl_interfaces::rcl_interfaces__rosidl_typesupport_introspection_cpp;rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp;rmw::rmw;rmw_implementation::rmw_implementation;rcl_logging_spdlog::rcl_logging_spdlog;/opt/ros/foxy/lib/librcl_lifecycle.so;/opt/ros/foxy/lib/liblifecycle_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/liblifecycle_msgs__rosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/liblifecycle_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/liblifecycle_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/liblifecycle_msgs__rosidl_typesupport_cpp.so;/opt/ros/foxy/lib/librclcpp.so;-lpthread;libstatistics_collector::libstatistics_collector;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_generator_c;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_typesupport_introspection_c;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_typesupport_c;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_generator_cpp;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp;libstatistics_collector::libstatistics_collector_test_msgs__rosidl_typesupport_cpp;rcl::rcl;rcl_yaml_param_parser::rcl_yaml_param_parser;rosgraph_msgs::rosgraph_msgs__rosidl_generator_c;rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_introspection_c;rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_c;rosgraph_msgs::rosgraph_msgs__rosidl_generator_cpp;rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_introspection_cpp;rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp;statistics_msgs::statistics_msgs__rosidl_generator_c;statistics_msgs::statistics_msgs__rosidl_typesupport_introspection_c;statistics_msgs::statistics_msgs__rosidl_typesupport_c;statistics_msgs::statistics_msgs__rosidl_generator_cpp;statistics_msgs::statistics_msgs__rosidl_typesupport_introspection_cpp;statistics_msgs::statistics_msgs__rosidl_typesupport_cpp;tracetools::tracetools;/opt/ros/foxy/lib/librclcpp_action.so;action_msgs::action_msgs__rosidl_generator_c;action_msgs::action_msgs__rosidl_typesupport_introspection_c;action_msgs::action_msgs__rosidl_typesupport_c;action_msgs::action_msgs__rosidl_generator_cpp;action_msgs::action_msgs__rosidl_typesupport_introspection_cpp;action_msgs::action_msgs__rosidl_typesupport_cpp;rcl_action::rcl_action;/opt/ros/foxy/lib/librclcpp_lifecycle.so;rclcpp::rclcpp;rcl_lifecycle::rcl_lifecycle;lifecycle_msgs::lifecycle_msgs__rosidl_generator_c;lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_introspection_c;lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_c;lifecycle_msgs::lifecycle_msgs__rosidl_generator_cpp;lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_introspection_cpp;lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp;/opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so;/opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so;std_msgs::std_msgs__rosidl_generator_c;/opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so;/opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so;builtin_interfaces::builtin_interfaces__rosidl_generator_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_c;builtin_interfaces::builtin_interfaces__rosidl_generator_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp;/opt/ros/foxy/lib/librcutils.so;dl;/opt/ros/foxy/lib/librcpputils.so;/opt/ros/foxy/lib/librosidl_typesupport_c.so;rcpputils::rcpputils;/opt/ros/foxy/lib/librosidl_typesupport_cpp.so;/opt/ros/foxy/lib/librosidl_runtime_c.so;/opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so;/opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so;/opt/ros/foxy/lib/libstd_srvs__rosidl_generator_c.so;rcutils::rcutils;/opt/ros/foxy/lib/libstd_srvs__rosidl_typesupport_introspection_c.so;std_srvs::std_srvs__rosidl_generator_c;rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c;/opt/ros/foxy/lib/libstd_srvs__rosidl_typesupport_c.so;rosidl_typesupport_c::rosidl_typesupport_c;/opt/ros/foxy/lib/libstd_srvs__rosidl_typesupport_introspection_cpp.so;rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp;/opt/ros/foxy/lib/libstd_srvs__rosidl_typesupport_cpp.so;rosidl_runtime_c::rosidl_runtime_c;rosidl_runtime_cpp::rosidl_runtime_cpp;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_interface::rosidl_typesupport_interface;geometry_msgs::geometry_msgs__rosidl_generator_c;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c;geometry_msgs::geometry_msgs__rosidl_typesupport_c;geometry_msgs::geometry_msgs__rosidl_generator_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_cpp;orocos-kdl;tf2::tf2;tf2_ros::tf2_ros;tf2_ros::static_transform_broadcaster_node;rviz_common::rviz_common"
)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/nav2_rviz_pluginsExport-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
