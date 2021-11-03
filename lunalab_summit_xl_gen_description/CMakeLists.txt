cmake_minimum_required(VERSION 3.5)
project(lunalab_summit_xl_gen_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY config launch lunalab_summit_xl_gen/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
