cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Remove the old installed entry point when upgrading an existing workspace.
# No other bringup executable or source file is touched.
install(CODE [[
  file(REMOVE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/bringup/ego_depth_inputs.py")
]])

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
