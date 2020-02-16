add_library(manual_loop_functions MODULE
        manual_qtuser_functions.h
        manual_qtuser_functions.cpp)

target_link_libraries(manual_loop_functions
  footbot_manual
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
