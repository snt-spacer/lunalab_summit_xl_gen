cmake_minimum_required(VERSION 3.5)
project(lunalab_summit_xl_gen_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Setup environment hooks that automatically prepend IGN_GAZEBO_RESOURCE_PATH and SDF_PATH when sourcing local_setup.<ext>
ament_environment_hooks(
  "cmake/environment_hooks/ign_gazebo_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

# Install directories
install(DIRECTORY config launch lunalab_summit_xl_gen lunalab_summit_xl_gen/materials lunalab_summit_xl_gen/meshes rviz urdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
