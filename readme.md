# Warehouse_Robots-808XFP

[![Build Status](https://github.com/adithyagaurav/Human_Detection_Tracker/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/adithyagaurav/Human_Detection_Tracker/actions/workflows/build_and_coveralls.yml)
[![Coverage Status](https://coveralls.io/repos/github/adithyagaurav/Human_Detection_Tracker/badge.svg?branch=master)](https://coveralls.io/github/adithyagaurav/Human_Detection_Tracker?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)





# ENPM 808X Final Project 

# WAREHOUSE ROBOT

# Team Members
 - **Driver** : Rishabh Singh (UID - 117511208)
 - **Navigator** : Adithya Singh (UID - 117507047)
 - **UML Design Kepeer** : Divyansh Agarwal (UID - 117730692)

## Overview

The idea is related to collaborative robots or “cobots”. The mobile robots are designed to
help human workers perform diverse tasks in warehouse environments and may also have the 
capability of acting as a mobile storage bin for picked orders. They also have the capability
to navigate autonomously using various object detection, path planning, and perception
techniques. We are proposing to add these Autonomous Mobile Robots (AMRs) to the
portfolio of Acme Robotics

##### Algorithms to be utilised
1. Monte Carlo Localization: Used by the
ROS MoveBase package to localize the
robot in the map
2. A* algorithm: Used by ROS MoveBase
to plan paths
3. Perspective Projection: Used by ROS
ArUco detector package.

## UML Class Diagram

 !["Class Diagram"](UML/Initial/UML.jpeg)



## Demo Output

#### Image Output

#### Video Output

=======
## Sprints and Backlogs
For this project Agile Iterative Process techniques were followed.
Sprint URL : [Sprint](https://docs.google.com/document/d/1i9uSZZQ_sIx2-5LxqOqx3VeSwF30iu6E4vHWd2NctFc/edit?usp=sharing)
Product & Interation Backlog : [ProductBCK](https://docs.google.com/spreadsheets/d/1FPVZE-TKWvhRZL-aQ01Hb0Aypjf1ur8BYXQdcFjWHR4/edit?usp=sharing_eil_se_dm&ts=638a8056)

## Phase 1


## Phase 2


## Standard install via command-line

```
git clone --recursive https://github.com/adithyagaurav/Human_Detection_Tracker
cd <path to repository>
chmod +x requirements.sh
./requirements.sh
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```

## Building for code coverage 
```
sudo apt-get install lcov
cmake -D COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug ../
make
make code_coverage
```
This generates a index.html page in the build/coverage sub-directory that can be viewed locally in a web browser.

## Working with Eclipse IDE ##

## Installation

In your Eclipse workspace directory (or create a new one), checkout the repo (and submodules)
```
mkdir -p ~/workspace
cd ~/workspace
git clone --recursive https://github.com/dpiet/cpp-boilerplate
```

In your work directory, use cmake to create an Eclipse project for an [out-of-source build] of cpp-boilerplate

```
cd ~/workspace
mkdir -p boilerplate-eclipse
cd boilerplate-eclipse
cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_ECLIPSE_VERSION=4.7.0 -D CMAKE_CXX_COMPILER_ARG1=-std=c++14 ../cpp-boilerplate/
```

## Import

Open Eclipse, go to File -> Import -> General -> Existing Projects into Workspace -> 
Select "boilerplate-eclipse" directory created previously as root directory -> Finish

# Edit

Source files may be edited under the "[Source Directory]" label in the Project Explorer.


## Build

To build the project, in Eclipse, unfold boilerplate-eclipse project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.

## Run

1. In Eclipse, right click on the boilerplate-eclipse in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. shell-app, cpp-test for unit testing)


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the boilerplate-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. shell-app).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project. If you're interested in this, try it out yourself and contact me on Canvas.
