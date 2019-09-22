cmake_minimum_required(VERSION 3.5)
project(service_tutorial)

# find dependencies
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/CalcTwoFloats.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files})

ament_export_dependencies(rosidl_default_runtime)

