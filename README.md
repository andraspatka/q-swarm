
# Setting up the development environment

## Setting up ArgOS

### Download and install ARGoS

You can find instructions for downloading and installing ARGoS here:

https://www.argos-sim.info/core.php

**Note:** It's important to install the correct package for your distribution. Otherwise you might get an error when trying to run experiments which work with loop functions:

```
[FATAL] Error initializing loop functions
[FATAL] Symbol "foraging_loop_functions" not found
```

### Download ARGoS examples

https://www.argos-sim.info/examples.php

### Packages for being able to compile ARGoS

Before installing Argos3

```bash
sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev lua5.3

#In case of error: sudo apt --fix-broken install
```

### Installing Fido: C++ Reinforcement learning library

Clone the following repository: https://github.com/andraspatka/Fido (Forked from FidoProject/Fido).

Checkout the release/NoSimulator branch.

Before running **make**, be sure to add the following compiler option in the **Makefile**: **-fPIC**

The first line of the **Makefile** should look like this:

```
CXXFLAGS += -std=c++11 -fPIC
```

Then just simply: 
```
sudo make install
```

### Downloading and compiling the examples

Follow this manual: https://www.argos-sim.info/user_manual.php

## Setting up Jetbrains CLion for development

Download CLion from the following link: https://www.jetbrains.com/clion/download/

**Note:** Clion is a *premium* Jetbrains product, meaning that you can't download it and use it for free beyond a 30 day trial. However, students and teachers can use it for free. Learn more at: https://www.jetbrains.com/student/


Choose **Open** then navigate to the *experiments* directory.

**Note:** A number of debugging configurations will automatically be detected.

Go to: **Run** -> **Edit configurations**

*Optionally* you can delete the configurations which end with **autogen** and **loop_functions**. These will not be needed since if you configure debugging for an experiment which uses loop functions, you can just simply set breakpoints in the loop function source code and the debugging process will function as expected.

To set up a debugging configuration, choose one from the list under **CMake Application** and then set the following fields:

 - **Executable** -> **Select Other** -> *select the ARGoS executable (Ubuntu default usr/bin/argos3)*
 - **Program arguments**: -c scenes/\<name of the scene\>.argos
 - **Working directory**: Input the project root path

Note: The **Working Directory** field is optional. You can leave it blank, but then you will have to fill the **Program arguments** section with the absolute path to the experiment file, such as: -c \<absolute_path>.argos

You can set up the **run configurations** the same way.

**Note:** If you want to build and run the project from CLion, then you need to make some changes to the .argos file. 
In each *library* attribute, replace the root folder **build** with **cmake-build-debug**

So basically the root folder in the *library* attribute should be:

 - **build** if you want to build and run with the help of qswarm.sh
 - **cmake-build-debug** if you want to build and run from CLion

### Recommendation: Add *.argos as an xml file association

Files with .argos file extensions use the XML structure. It makes editing them a whole lot easier, if the editor recognizes them as XML files.

To do this in CLion, do the following:

 - Go to **File** -> **Settings** (ctrl+alt+s)
 - Go to **Editor** -> **File Types**
 - Choose **XML** from the **Recognized File Types** list
 - Press the **+** button in the **Registered Patterns** pane
 - Type in **\*.argos**, press **OK**

Now CLion recognizes .argos files as XML. You now get syntax highlighting, auto complete and hints.

### Possible errors

```
Couldn't load scenes/single_footbot <ticpp.cpp@823>
Description: Failed to open file
File: scenes/single_footbot
Line: 0
Column: 0
```

**Solution:** Check if you specified the path to the experiment correctly. Maybe you forgot to add the **.argos** file extension to the end of the experiment name.

```
[FATAL] No --help, --version, --config-file or --query options specified.
```

**Solution:** Check if you specified the **Program arguments** correctly.

# Directory structure - what to find where

### docs

Contains the documentation.

### experiments

Contains all of the files needed for building and running an ARGoS experiment.

### measurements

Contains the logs from the experiments.

### proto

Contains algorithm prototypes written in higher level languagse (JavaScript, Python).

### literature

Contains scientific papers regarding the thesis.