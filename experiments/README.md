# Running the experiments

A build script: ```qswarm.sh``` was created to make building, running and debugging the experiments easier.

Usage:
```bash
# For building the project type, use
./qswarm.sh build

# For running an experiment, use:
# The experiment_name should be specified without the argos
# file extension.
./qswarm.sh run <experiment_name>

# Performs a build and runs the given experiment
./qswarm.sh build-run <experiment_name>

# Performs a build with the cmake -DCMAKE_BUILD_TYPE=Debug option.
# This makes it possible to debug the code with gdb
./qswarm.sh build-debug <experiment_name> 
```

Directory structure:

 - controllers: 
    - contains the logic with which the robot operates
 - loop_functions: 
    - contains special hook functions to be able to customize the simulation
 - scenes: 
    - contains the *.argos experiment files which describes the simulation and references the required controller and loop function files

# Experiments

### diffusion_10: Argos3 example
### foraging: Argos3 example
### trajectory: Argos3 example
### single_footbot: 
One footbot that can do some basic obstacle avoidance, albeit in quite a strange manner. 

Inspired from diffusion_10 controller example written by Carlo Pinciroli. 

The example shows how to work with vectors using the Argos3 framework.

[Description of the experiment](https://github.com/andraspatka/q-swarm/blob/master/docs/environment.md)
