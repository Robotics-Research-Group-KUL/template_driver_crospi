cmake_minimum_required(VERSION 3.8)
project(template_driver_crospi)

message("--------------template_driver_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)

find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")


include_directories(
   include
   ${Boost_INCLUDE_DIRS}
)


set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

set(src_template_driver_crospi
    src/template_driver_crospi.cpp
)

add_library(template_driver_crospi 
            SHARED
            ${src_template_driver_crospi})

set(MY_EXTERNAL_LIB "") #sets it empty such that it can be redefined later to include external lib


# UNCOMENT THE FOLLOWING LINES AND MODIFY IF AN EXTERNAL LIBRARY IS DESIRED TO BE INCLUDED WITHIN THIS PACKAGE AND COMPILED BEFORE THE REST OF THE PACKAGE AS A DEPENDENCY
# set(MY_EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/)
# set(MY_EXTERNAL_LIB_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/include)
# set(MY_EXTERNAL_LIB_DIR_SO_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/external/my_external_lib/lib)

# message("--------------MY_EXTERNAL_LIB_DIR: ${MY_EXTERNAL_LIB_DIR}----------------")

# add_custom_target(external_compiled_lib COMMAND make clean && make
#                    WORKING_DIRECTORY ${MY_EXTERNAL_LIB_DIR}/build/GNUMake
# )


# include_directories(${MY_EXTERNAL_LIB_INCLUDE})

# add_dependencies(template_driver_crospi external_compiled_lib) #Makes sure to compile first external_compiled_lib before the template_driver_crospi

# message("-------------- MY_EXTERNAL_LIB_DIR compiled----------------")


# find_library(MY_EXTERNAL_LIB
#              NAMES <name_of_the_so_lib>.so
#              PATHS ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}
# )

# message("--------------MY_EXTERNAL_LIB: ${MY_EXTERNAL_LIB}----------------")



set(rosdependencies
"crospi_core"
"pluginlib"
)

# set(etasl_task_utils_DIR /workspaces/devcontainer/build/crospi_core/src/misc/etasl_task_utils)

# find_library(etasl_task_utils
#              NAMES libetasl_task_utils.so
#              PATHS ${etasl_task_utils_DIR}
# )
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/path/to/etasl_task_utils")


# find_library(etasl_task_utils
#               NAMES libetasl_task_utils.so)

# find_library(robot_interfacing_utils
#               NAMES librobot_interfacing_utils.so)

# find_library(crospi_node_utils
#               NAMES libcrospi_node_utils.so)
# find_package(crospi_node_utils REQUIRED)
# find_package(robot_interfacing_utils REQUIRED)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
    # etasl_misc_lib 
    # etasl_task_utils
    # ${crospi_core_LIBRARIES}
    # etasl_interfacing_utils
    # crospi_node_utils
    # ${robot_interfacing_utils}
    # ${etasl_task_utils}
  )

pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(template_driver_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions. 
set(ros_targets "")
foreach(pkg IN LISTS rosdependencies)
  list(APPEND ros_targets "${pkg}::${pkg}") #This might not work for all packages. ${pkg_TARGETS} might be needed for some. Check https://github.com/ament/ament_cmake/issues/584#issuecomment-2931061571
endforeach()

if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(template_driver_crospi ${rosdependencies})
      target_link_libraries(template_driver_crospi ${dependencies})
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(template_driver_crospi  ${dependencies} ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()


ament_export_dependencies(crospi_core)

ament_export_libraries(
  template_driver_crospi
)

# install(TARGETS template_driver_crospi
#   DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS template_driver_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas
  DESTINATION share/${PROJECT_NAME})
  
# ament_export_include_directories(
#   include
# )





# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
