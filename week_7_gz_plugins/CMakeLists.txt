cmake_minimum_required(VERSION 3.16)
project(week_7_gz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# SDF refers to the plugin by library name: filename="gz_model_attacher" loads
# libgz_model_attacher.so
add_library(gz_model_attacher SHARED src/ModelAttacher.cc)
target_compile_features(gz_model_attacher PRIVATE cxx_std_17)
target_link_libraries(gz_model_attacher gz-sim::gz-sim gz-plugin::register)
install(TARGETS gz_model_attacher LIBRARY DESTINATION lib)

# Sourcing install/setup.bash adds <prefix>/lib to GZ_SIM_SYSTEM_PLUGIN_PATH, so the Gazebo
# server can find the plugin.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
