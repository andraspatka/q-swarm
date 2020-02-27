include_directories(${CMAKE_SOURCE_DIR})

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(manual_control)
  add_subdirectory(id)
endif(ARGOS_COMPILE_QTOPENGL)
