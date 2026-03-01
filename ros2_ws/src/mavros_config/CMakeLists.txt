cmake_minimum_required(VERSION 3.8)
project(mavros_config)

find_package(ament_cmake REQUIRED)

# Install all YAMLs in config/ into share/<pkg>/config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()