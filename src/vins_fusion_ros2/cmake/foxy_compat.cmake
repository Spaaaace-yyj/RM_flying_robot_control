# This package declares its own policies with cmake_minimum_required(). Keep
# pre-3.5 policy declarations in external Focal/Foxy dependencies (including
# Ceres 1.14 and googletest) usable with CMake 4, which removed that compatibility.
# Use a directory-scoped variable: no forced cache entry or dependency edits.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR
     CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS "3.5")
    set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
  endif()
endif()
