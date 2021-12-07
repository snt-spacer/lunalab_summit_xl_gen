cmake_minimum_required(VERSION 3.5)
project(lunalab_summit_xl_gen_control)

# Find dependencies
find_package(catkin REQUIRED)
find_package(lunalab_summit_xl_gen_description REQUIRED)

# Install directories
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Setup the project
catkin_package()
