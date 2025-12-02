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

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "/opt/boost/1.89.0")
set(CMAKE_PREFIX_PATH "/opt/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

# -- end -- Stuff for triple buffer implementation of boost 1.89

include_directories(
   include
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

# message(--------- jfjjfjfj ${CMAKE_SOURCE_DIR})

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

ament_target_dependencies(template_driver_crospi ${rosdependencies})  
target_link_libraries(template_driver_crospi ${dependencies})

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
