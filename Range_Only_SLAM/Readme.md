# Range-only SLAM

 Here,  the concepts of constraints and interval analysis on a concrete Simultaneous Localization and Mapping (SLAM) problem is applied.

## Instructions

To run the code you need to install  [Tubex](http://simon-rohou.fr/research/tubex-lib/doc/index.html) for solving interval tubes and the graphical tool  [VIbes](http://simon-rohou.fr/research/tubex-lib/doc/install/01-installation.html#graphical-tools) viewer.  To do so, see the instructions [here](https://github.com/jad-rabehi/Constraint-programming#instructions).



### Launch

Launch the graphical viewer first using:
```bash
  VIBes-viewer
```
---
Then, launch the code with:
```bash
  cd build
  cmake ..
  make
  ./Range_Only_SLAM
```



You should obtain these figures:

<p align="center">
  <img width="750" src="images/deadreckoning.png">
</p>

* the fist figure shows the deadrecking localization based on the knowledge of initial position the evolution of the robot

<p align="center">
<img  src="images/Range_SLAM.png" width=750>
<p/>

* the second figure shows the SLAM using only range measurements, in which intervals for both robot pose and beacons positions are obtained (contrated) using contractors





