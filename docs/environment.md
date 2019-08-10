# Single footbot obstacle avoidance using push and pull forces

The experiment contains a single footbot, obstacles and an end destionation. The footbot strives towards avoiding collisions with obstacles and proceeds towards the end destination.

The scene is a rectangular arena with obstacles, an end goal and a starting position.

The agent is a single footbot.

**Proximity sensors** are used for sensing, and **differential steering actuators** are the actuators.

## PEAS - Performance Enviroment Actuators Sensors

Specification of the task environment, using the PEAS aspects.

### Performance

 - Avoiding collisions
 - Proceeding towards the end goal
 - Speed

### Environment

 - Rectangular arena
 - Obstacles
 - End goal

### Actuators

 - Differential steering actuator

### Sensors

 - Proximity sensor

The environment can be considered as **full observable** since the "sensors detect all aspects that are *relevant* to the choice of action" (Peter Norvig and Stuart Russel - Artificial Intelligence: A Modern Approach).

The environment is also deterministic, sequential, static, continuous and known.