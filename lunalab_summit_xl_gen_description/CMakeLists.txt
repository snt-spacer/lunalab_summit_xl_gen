cmake_minimum_required(VERSION 3.5)
project(lunalab_summit_xl_gen_description)

# Find dependencies
find_package(catkin REQUIRED)

# Install directories
install(DIRECTORY config lunalab_summit_xl_gen/materials lunalab_summit_xl_gen/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Setup the project
catkin_package()
