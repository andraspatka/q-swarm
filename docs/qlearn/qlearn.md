# Obstacle avoidance while progressing towards a goal using Fido RL library (Wire fitted Q-Learning)

## Definition of states

The same is true for the light sensor, as for the proximity sensor: 

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

To reduce complexity, as of now, only four states are defined:

 - The first is the vector sum of the sensor values of sensors 0 - 5
 - The second is the vector sum of the sensor values of sensors 6 - 11
 - The third is the vector sum of the sensor values of sensors 12 - 17
 - The fourth is the vector sum of the sensor values of sensors 18 - 23

## Definition of actions

The used actuators are differential steering actuators which can take two values:

 - if both values are equal, the agent moves forward
 - if the second value is higher the agent turns left
 - if the first value is higher the agent turns right

With this in mind, the defined actions are:

 - min action: -2.5, -2.5
 - max action: 2.5, 2.5
 - base of dimension: 3 (3 discrete values between min action and max action)

## Rewards

The reward is the negative of the distance to the goal. SO the positive of the light sensor's maximal reading value.