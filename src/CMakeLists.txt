cmake_minimum_required(VERSION 3.16)
project(long_range_constraints CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(long-range-constraints
  long-range-constraints.cpp
  ChainSceneBase.cpp
  RigidChainSim.cpp
  DrawUtils.cpp
  ChainLRCScene.cpp
  ChainAngleBoundsScene.cpp
)

target_include_directories(long-range-constraints PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# GLM is header-only; provide include path via your toolchain/vcpkg as needed.

if (WIN32)
  # If you use vcpkg, link packages there. Otherwise set your GLUT/OpenGL libs.
  target_link_libraries(long-range-constraints PRIVATE opengl32 glu32)
elseif(APPLE)
  find_library(OpenGL_LIBRARY OpenGL)
  find_library(GLUT_LIBRARY GLUT)
  target_link_libraries(long-range-constraints PRIVATE ${OpenGL_LIBRARY} ${GLUT_LIBRARY})
else()
  find_package(OpenGL REQUIRED)
  target_link_libraries(long-range-constraints PRIVATE OpenGL::GL OpenGL::GLU)
  # Add GLUT/freeglut as needed.
endif()
