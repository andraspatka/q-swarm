
# Setting up the development environment

## Setting up ArgOS

### Download and install ARGoS

You can find instructions for downloading and installing ARGoS here:

https://www.argos-sim.info/core.php

### Download ARGoS examples

https://www.argos-sim.info/examples.php

### Packages for being able to compile ARGoS

I installed all of the packages which are required for compiling ARGoS. Most probably not all of them all needed for simply running the examples.

```
sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.2-dev \
  lua5.2 doxygen graphviz graphviz-dev asciidoc
```

### Downloading and compiling the examples

Follow this manual: https://www.argos-sim.info/user_manual.php

### Fixing the errors when trying to run the examples - installing qt 5.9

```
wget http://download.qt.io/official_releases/qt/5.9/5.9.1/qt-opensource-linux-x64-5.9.1.run

chmod +x qt-opensource-linux-x64-5.9.1.run

./qt-opensource-linux-x64-5.9.1.run

After this, follow the installation wizard.
```

Now when you try to run the examples with:
```
argos3 -c experiments/diffusion_1.argos
```
You will probably get some errors messages which look something like this:

```
[FATAL] Can't load library "/usr/lib/argos3/libargos3plugin_simulator_footbot.so" even after trying to add extensions for shared library (so) and module library (so): 
/usr/lib/argos3/libargos3plugin_simulator_footbot.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5: version `Qt_5' not found (required by /usr/lib/argos3/libargos3plugin_simulator_qtopengl.so)
/usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: /usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: cannot open shared object file: No such file or directory
/usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: OK
```

To fix this, copy the following files from your QT installation directory **5.9.1/gcc_64/lib/** to **usr/lib/x86_64-linux-gnu** directory:
 
 - libicudata.so.56  
 - libicui18n.so.56  
 - libicuuc.so.56  
 - libQt5Core.so.5  
 - libQt5Gui.so.5  
 - libQt5Widgets.so.5

## Setting up Eclipse IDE for ARGoS

Download Eclipse IDE: https://www.eclipse.org/downloads/

Install for C++.

Optionally install CMake editor from the Eclipse Marketplace.

Genetic algorithms library
```
sudo apt-get install libga-dev
```

