cmake_minimum_required( VERSION 3.5 )

# Require C++17 (or later)
set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Create Project
project(rs_icosaf)
add_executable(rs_icosaf example.hpp rs-icosaf.cpp)

# Set StartUp Project
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "rs_icosaf")

# Find Package
# librealsense2
find_package(realsense2 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL 1.7 REQUIRED)

# GLFW
find_package(glfw3 3.3 REQUIRED)
#OPENGL
find_package(OpenGL REQUIRED)

#pthread
find_package(Threads)

if(realsense2_FOUND AND OpenCV_FOUND AND PCL_FOUND)
  # Additional Include Directories
  include_directories( ${realsense_INCLUDE_DIR} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  target_include_directories(rs_icosaf PUBLIC ${OPENGL_INCLUDE_DIR})

  # Additional Dependencies
  target_link_libraries(rs_icosaf ${realsense2_LIBRARY})
  target_link_libraries(rs_icosaf ${OpenCV_LIBS})
  target_link_libraries(rs_icosaf ${PCL_LIBRARIES})
  target_link_libraries(rs_icosaf glfw)
  target_link_libraries(rs_icosaf ${OPENGL_gl_LIBRARY})
  target_link_libraries(rs_icosaf ${OPENGL_glu_LIBRARY})
  target_link_libraries (rs_icosaf ${CMAKE_THREAD_LIBS_INIT})
  if( NOT WIN32 )
      target_link_libraries(rs_icosaf stdc++fs)
  endif()
endif()

