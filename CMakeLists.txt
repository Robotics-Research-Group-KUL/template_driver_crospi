cmake_minimum_required(VERSION 3.8)
project(template_driver_etasl)

message("--------------template_driver_etasl compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# find_package(ament_index_cpp REQUIRED)

find_package(pluginlib REQUIRED)
find_package(etasl_ros2 REQUIRED)

include_directories(
   include
)

set(CMAKE_CXX_STANDARD 17)

# include_directories(include ${LUA_INCLUDE_DIR})

set(src_template_driver_etasl
    src/template_driver_etasl.cpp
)

add_library(template_driver_etasl 
            SHARED
            ${src_template_driver_etasl})

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

# add_dependencies(template_driver_etasl external_compiled_lib) #Makes sure to compile first external_compiled_lib before the template_driver_etasl

# message("-------------- MY_EXTERNAL_LIB_DIR compiled----------------")


# find_library(MY_EXTERNAL_LIB
#              NAMES <name_of_the_so_lib>.so
#              PATHS ${MY_EXTERNAL_LIB_DIR_SO_LOCATION}
# )

# message("--------------MY_EXTERNAL_LIB: ${MY_EXTERNAL_LIB}----------------")



set(rosdependencies
"etasl_ros2"
"pluginlib"
)

# set(etasl_task_utils_DIR /workspaces/devcontainer/build/etasl_ros2/src/misc/etasl_task_utils)

# find_library(etasl_task_utils
#              NAMES libetasl_task_utils.so
#              PATHS ${etasl_task_utils_DIR}
# )
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/path/to/etasl_task_utils")

# message(--------- jfjjfjfj ${CMAKE_SOURCE_DIR})

# find_library(etasl_task_utils
#               NAMES libetasl_task_utils.so)

# find_library(robot_interfacing_utils
#               NAMES librobot_interfacing_utils.so)

# find_library(etasl_node_utils
#               NAMES libetasl_node_utils.so)
# find_package(etasl_node_utils REQUIRED)
# find_package(robot_interfacing_utils REQUIRED)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
    # etasl_misc_lib 
    # etasl_task_utils
    # ${etasl_ros2_LIBRARIES}
    # etasl_interfacing_utils
    # etasl_node_utils
    # ${robot_interfacing_utils}
    # ${etasl_task_utils}
  )

pluginlib_export_plugin_description_file(etasl_ros2 plugins.xml)


target_include_directories(template_driver_etasl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(template_driver_etasl ${rosdependencies})  
target_link_libraries(template_driver_etasl ${dependencies})

ament_export_dependencies(etasl_ros2)

ament_export_libraries(
  template_driver_etasl
)

# install(TARGETS template_driver_etasl
#   DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS template_driver_etasl
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
