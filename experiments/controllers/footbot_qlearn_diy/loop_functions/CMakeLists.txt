include_directories(${CMAKE_SOURCE_DIR})

# Descend into the foraging_loop_functions directory
add_subdirectory()

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory()
endif(ARGOS_COMPILE_QTOPENGL)
