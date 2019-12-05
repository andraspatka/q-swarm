# Single/multiple footbot obstacle avoidance using push and pull forces

The experiment contains a single/multiple footbot(s), obstacles and an end destionation. The footbot strives towards avoiding collisions with obstacles and proceeds towards the end destination.

The scene is a rectangular arena with obstacles, an end goal and a starting position.

The agent is a single footbot.

**Proximity and light sensors** are used for sensing, and **differential steering actuators** are the actuators.

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
 - Light sensor

The environment can be considered as **fully observable** since the "sensors detect all aspects that are *relevant* to the choice of action" (Peter Norvig and Stuart Russel - Artificial Intelligence: A Modern Approach).

The environment is also deterministic, sequential, static, continuous and known.

The proximity sensors are used for sensing obstacles, the light sensors are used for detecting the end destination, which is modelled with a light.

## Description of the obstacle avoidance logic

The agent has proximity sensors around its case. From each proximity sensor an angle and length value can be read. These can be converted to a physical vector. 

The proximity sensors are layed out in the following fashion (javadoc of the footbot proximity sensor header file):

```c++
/**
 * @file <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
 *
 * @brief This file provides the definition of the footbot proximity sensor.
 *
 * This file provides the definition of the footbot proximity sensor.
 * The sensors are evenly spaced on a ring around the base of the robot.
 * Therefore, they do not turn with the turret. The readings are normalized
 * between 0 and 1, and are in the following order (seeing the robot from TOP,
 * battery socket is the BACK):
 *
 *              front
 *               
 *               0 23
 *             1     22
 *           2         21
 *         3             20      r
 * l     4                 19    i
 * e   5                     18  g
 * f   6                     17  h
 * t     7                 16    t
 *         8             15
 *           9         14
 *            10     13
 *              11 12
 *
 *              back
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */
```

The angles from the readings can be in the [-180, 180] interval, where:
 
 - Sensors 0 - 11 represent the [0, 180] interval (positive)
 - Sensors 12 - 23 represent the [-180, 0) interval (negative)

The sum of all of these vectors (from now on: **vecSum**) can be used to determine what action the agent should take. The **vecSum**'s angle can tell us where the obstacle is detected.

Deciding when to turn left or right can be decided rather easily, from the **vecSum**'s angle:

 - If the angle of the **vecSum** is negative (obstacle to the right), then the agent should turn left.
 - If the angle of the **vecSum** is positive (obstacle to the left), then the agent should turn right.

Deciding when it is okay to simply move forward is more of a challenge. 

This decision could be made based on **vecSum**'s length value (if it is large enough, then the obstacle is far away, and it is okay to move forward) and angle value (if the absolute value of the angle is big enough, then the obstacle is not located in front of the robot, and it is okay to move forward).

Based on these decisions, when the agent encounters an obstacle, its *sequence of action* is to turn around.
This can lower the *performance measure*, since the agent should progress towards an end goal.

That is why the condition to move forward was revised:

 - If the dot product between **vecSum** and **straight** vector is smaller than *EPSILON*, then go forward

Where the **straight** vector is a vector that always points forward (when the end goal is not being detected).
What this condition means, is that the angle between the **vecSum** and **straight** vectors should be >= 90 degrees.

With this condition, when the agent encounters a *wall* obstacle, then its *sequence of action* is to turn (to the right or to the left) while it is not parallel to the obstacle. Once it is parallel to the obstacle, it proceeds to go forward.

Instead of turning around, it goes along the side of the obstacle.
This helps to somewhat preserve the *performance measure*, but information about the end goal is needed, in order to decide in which direction to turn.
This information is provided by the *light sensors*.

## Progressing towards the destination

The destination is defined as a light, and is detected by the agent's light sensors. This aditional sensor is implemented similarly to the proximity sensor. It also consists of 24 sensors, located in a ring around the footbot.
In the previous section, the readings from the proximity sensor where summed together. These vectors were considered a **pushing force**. The readings of the light sensor should be considered in an opposite manner. Since the goal is to arrive at the destination, these vectors should be subtracted from **vectorSum**. They are a **pulling force**.

Considering readings of the light sensor as a pushing force would result in the agent progressing in the complete opposite direction.

Introducing the light sensor, does require revising the *moving forward condition* a bit. The change comes in the form of the **straight vector**. When the destination is detected, then the **straight vector** is equal to **vecSum**, thus making the angle between the two vectors virtually 0.
When this small change is not implemented, then when the agent reaches the end destination, rather than moving towards it, it just circles around it.

Basically, when the agent does not detect the light, then it should move along the side of the obstacle, then once it detects the light again, it should move towards it.

The **push_pull** directory contains three videos of the agents in action. 

