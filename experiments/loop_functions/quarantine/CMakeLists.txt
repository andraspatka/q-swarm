add_library(quarantine_loop_functions MODULE
        quarantine_qtuser_functions.h
        quarantine_qtuser_functions.cpp)

target_link_libraries(quarantine_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})
