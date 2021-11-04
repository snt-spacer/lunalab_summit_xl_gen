cmake_minimum_required(VERSION 3.5)
project(lunalab_summit_xl_gen_ign)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(lunalab_summit_xl_gen_description REQUIRED)
find_package(lunalab_summit_xl_gen_moveit_config REQUIRED)

# Install directories
install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
