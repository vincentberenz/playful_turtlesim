# Behavior design of a turtlesim using Playful

[Turtlesim](http://wiki.ros.org/turtlesim) is a cute 2d simulated turtle famous for being used in ROS tutorials.

[Playful](https://github.com/vincentberenz/playful_tutorial/wiki/00.-Overview) is a software for robotic bevahior design. 

This repo shows the control of a turtlesim via Playful, for educational purposes.

## How to

### pre-requisits

ROS and turtlesim must be installed. Any version.

### Playful

```bash
pip install funcsigs # only if you are using python2.7
pip install playful
```

### Running the demo

- start ROS core

```bash
roscore
```

- in another terminal, start turtlesim

```bash
rosrun turtlesim turtlesim_node
```
- in another terminal, clone this repository

```bash
git clone https://github.com/vincentberenz/playful_turtlesim.git
```
- start playful

```bash
cd playful_turtlesim
playful execute
```

(press 'q' to exit)

You should see your turtle chasing any other turtles going close to its home (a virtual point a the bottom of the simulated area).

## Check the code

- The playful script that is playful_turtlesim/play/program.py
- This script orchestrate the python code that is in playful_turtlesim/py/turtlesim_play.py

## Support 

playful-support .a.t. tuebingen.mpg.de

## Author

Vincent Berenz, Max Planck Institute for Intelligent Systems






