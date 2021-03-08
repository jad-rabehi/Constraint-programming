# Localization with asynchonous measurements
---

## Instructions
To run the code you need  [Tubex](http://simon-rohou.fr/research/tubex-lib/doc/index.html) for solving interval tubes and the graphical tool  [VIbes](http://simon-rohou.fr/research/tubex-lib/doc/install/01-installation.html#graphical-tools) viewer.

** Installing Tubex and its dependencies  **
see [here](http://simon-rohou.fr/research/tubex-lib/doc/install/01-installation-full-linux.html#installing-tubex-on-linux-for-c-use)


** Installing graphical tool  **

On Linux systems, a fast installation can be made using the following command lines:
```bash
 sudo apt-get install qt5-default libqt5svg5-dev cmake git
 git clone https://github.com/ENSTABretagneRobotics/VIBES
 cd VIBES/viewer ; mkdir build ; cd build ; cmake .. ; sudo make install
```
On terminal,  launch it using
```bash
  VIBes-viewer
```

---

## Algorithm

Q-Learning is a model-free form of machine learning, in the sense that the AI "agent" does not need to know or have a model of the environment that it will be in. For a given environment, everything is broken down into "states" and "actions." The states are observations and samplings that we pull from the environment, and the actions are the choices the agent has made based on the observation. 


The algorithm has a function that calculates the quality of a state–action combination known as Q-table: <img src="https://render.githubusercontent.com/render/math?math=Q : S \times A \rightarrow R.">



Before learning begins, <img src="https://render.githubusercontent.com/render/math?math=Q"> is initialized to a random value. Then, at each time <img src="https://render.githubusercontent.com/render/math?math=t"> the agent selects an action <img src="https://render.githubusercontent.com/render/math?math=a_{t}">, 
observes a reward <img src="https://render.githubusercontent.com/render/math?math=r_{t}">,
 enters a new state <img src="https://render.githubusercontent.com/render/math?math=s_{t%2B1}">
 (that may depend on both the previous state <img src="https://render.githubusercontent.com/render/math?math=s_{t}">
 and the selected action), and <img src="https://render.githubusercontent.com/render/math?math=Q"> is updated. The core of the algorithm is a _Bellman equation_ as a simple _value iteration_ update, using the weighted average of the old value and the new information: 

 


<img src="images/qlearningwiki.png" width="750">
_figure credit: https://en.wikipedia.org/wiki/Q-learning_

where <img src="https://render.githubusercontent.com/render/math?math=r_{t}">
 is the reward received when moving from the state <img src="https://render.githubusercontent.com/render/math?math=s_{t}">
 to the state <img src="https://render.githubusercontent.com/render/math?math=s_{t%2B1}">.

For more details, the reader can see the reference
* Sutton, Richard S., and Andrew G. Barto. Reinforcement learning: An introduction. MIT press, 2018.
---

## Environment

The algorithm is applied to an OpenAI's gym, the "MountainCar-v0" environment. The goal is to learn the car to move from the initial (lower) position to  reach the flag (upper) position, as it is shown in figure below.

<img src="images/startepisode.png" width="350">  >> into >> <img src="images/endepisode.png" width="350">


---
### Copyright

