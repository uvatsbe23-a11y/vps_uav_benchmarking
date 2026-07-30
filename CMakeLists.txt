cmake_minimum_required(VERSION 3.8)
project(vps_uav_benchmarking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install configs and scripts
install(DIRECTORY config docs
  DESTINATION share/vps_uav_benchmarking
)

install(PROGRAMS 
  scripts/flight_analysis.py
  DESTINATION lib/vps_uav_benchmarking
)

install(DIRECTORY launch
  DESTINATION share/vps_uav_benchmarking
)

ament_package()
