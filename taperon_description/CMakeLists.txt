cmake_minimum_required(VERSION 3.8)
project(taperon_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Absolute path so CMake never loses it
set(TAPERON_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/urdf/taperon.urdf.xacro)

# Generate URDF from Xacro and install it
xacro_add_files(
  ${TAPERON_XACRO}
  INSTALL DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes (adjust if your path differs)
install(DIRECTORY urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/urdf/meshes
)

# Install URDF, XACROs, and meshes so FindPackageShare works
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
    PATTERN "*.urdf"
    PATTERN "*.xacro"
    PATTERN "meshes/*"
)

ament_package()
