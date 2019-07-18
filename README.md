
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

I installed all of the packages which are required for compiling ARGoS. Most probably not all of them all needed for simply running the examples.

```
sudo apt-get install cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.2-dev \
  lua5.2 doxygen graphviz graphviz-dev asciidoc
```

### Optional step - installing GALIB: Genetic Algorithms Library

**Note:** Although the build finishes without a problem, running the argos3-examples which require GALIB result in a Segmentation fault error. Unfortunate. A solution for this problem is yet to be found.

First download distribution 247 from the following link: http://lancet.mit.edu/ga/dist/

**Extract**

**Cd into galib247**

*Note: I'm using Ubuntu 16.04 LTS with gcc 5.04*

This **makevars** file is configured in such a way, that only the compiler options for gcc3 and gcc4 are present (in the original makevars file there are lots of options for lots of different compilers). 

Also the option -fPIC is added to **CXXFLAGS**. Without this you get an error when trying to `make` the ARGoS examples.

Replace the contents of the **makevars** file with the following:

```
# -*- Mode: makefile -*-
# Copyright (c) 2005 Matthew Wall, all rights reserved
# makefile variables for compiling on unix environments
# -----------------------------------------------------------------------------

LIB   =libga.a

### Set these directories to whatever is appropriate for your system.  These 
### are used only if you do a 'make install'.  They specify where the library
### and header files should be installed.
DESTDIR=/usr
HDR_DEST_DIR=$(DESTDIR)/include
LIB_DEST_DIR=$(DESTDIR)/lib


### Make sure that these are ok for your operating system.
MKDEPEND    = makedepend
MKDIR       = mkdir -p
CP          = cp
RM          = rm -rf


### Uncomment a block from the list below appropriate for the compiler and 
### operating system on which you are compiling.

# Added the -fPIC option to CXXFLAGS (Position independent code). 
# The ARGoS examples can't be compiled without it.
# gcc3, gcc4
#  verified 28dec04 on linux-x86 (fedora core 2 with gcc 3.3.3)
#  verified 28dec04 on linux-ppc (yellow dog 3 with gcc 3.2.2)
#  verified 28dec04 on win2k-x86 (cygwin-win2k with gcc 3.3.3)
#  verified 10jan05 on linux-x86 (fedora core 3 with gcc 3.4.2)
#  verified 06mar07 on linux-x86 (debian with gcc 3.3.5)
#  verified 06mar07 on linux-x86 (ubuntu with gcc 4.0.3)
#  verified 06mar07 on macosx-ppc (macosx 10.4.8 with gcc 4.0.1)
CXX         = g++
CXXFLAGS    = -g -fPIC -Wall
LD          = g++ -w
AR          = ar rv
INSTALL     = install -c
RANLIB      = echo no ranlib
```

The **makefile** was changed in such a way that only the library is compiled. Compiling the examples is simply unneeded.
Replace the contents of the makefile with the following:

```makefile
# -*- Mode: makefile -*-
# Makefile for GAlib
# Copyright (c) 1996-2005 Matthew Wall, all rights reserved
#
# If you need to customize the build of galib, you should first modify the
# variables in the makevars file.

GALIB_VERSION=2.4.7
GALIB_VER=247
TMPDIR=/var/tmp
RELDIR=$(TMPDIR)/galib$(GALIB_VER)

all: lib

lib:
	cd ga; $(MAKE)

install:
	cd ga; $(MAKE) install

uninstall:
	cd ga; $(MAKE) uninstall

clean:
	cd ga; $(MAKE) clean

release: clean 
	rm -rf $(RELDIR)
	mkdir -p $(RELDIR)
	cp -rp * $(RELDIR)
	rm -rf `find $(RELDIR) -name CVS`
	rm -rf `find $(RELDIR) -name .svn`
	rm -f `find $(RELDIR) -name "*~"`
	echo $(GALIB_VERSION) > $(RELDIR)/VERSION
	perl -pi -e 's/evision: \d+\.\d+ /evision: $(GALIB_VERSION) /' $(RELDIR)/ga/gaversion.h
	perl -pi -e 'chop($$dt=`date +"%Y/%m/%d %H:%M:%S"`); s/Date: ..\/..\/.. ..:..:.. /Date: $$dt /' $(RELDIR)/ga/gaversion.h
	cd $(RELDIR)/..; tar cvfz galib$(GALIB_VER).tgz galib$(GALIB_VER) > $(TMPDIR)/galib$(GALIB_VER)-manifest-tar.txt
	cd $(RELDIR)/..; zip -r galib$(GALIB_VER).zip galib$(GALIB_VER) > $(TMPDIR)/galib$(GALIB_VER)-manifest-zip.txt
	@echo "  GAlib $(GALIB_VERSION) has been released to $(TMPDIR)"
```

Next run the following two commands:
```bash
# Builds the sources, creates libga.a
make
# Copies the header files to the specified include directory in makevars
# Copies libga.a to the specified library directory in makevars
sudo make install
```

### Downloading and compiling the examples

Follow this manual: https://www.argos-sim.info/user_manual.php

### Fixing the errors when trying to run the examples - installing qt 5.9



Running the examples:
```
argos3 -c experiments/diffusion_1.argos
```

You might get some errors messages which look something like this:

```
[FATAL] Can't load library "/usr/lib/argos3/libargos3plugin_simulator_footbot.so" even after trying to add extensions for shared library (so) and module library (so): 
/usr/lib/argos3/libargos3plugin_simulator_footbot.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5: version `Qt_5' not found (required by /usr/lib/argos3/libargos3plugin_simulator_qtopengl.so)
/usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: /usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: cannot open shared object file: No such file or directory
/usr/lib/argos3/libargos3plugin_simulator_footbot.so.so: OK
```

To fix this, first install qt 5.9

```bash
wget http://download.qt.io/official_releases/qt/5.9/5.9.1/qt-opensource-linux-x64-5.9.1.run

chmod +x qt-opensource-linux-x64-5.9.1.run

./qt-opensource-linux-x64-5.9.1.run

After this, follow the installation wizard.
```

Then copy the following files from your QT installation directory **5.9.1/gcc_64/lib/** to **usr/lib/x86_64-linux-gnu** directory:
 
 - libicudata.so.56  
 - libicui18n.so.56  
 - libicuuc.so.56  
 - libQt5Core.so.5  
 - libQt5Gui.so.5  
 - libQt5Widgets.so.5

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

Contains the documentation

### experiments

Contains all of the files needed for building and running an ARGoS experiment

### measurements

Contains the logs from the experiments.