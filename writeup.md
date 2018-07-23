## Path Planning
---

**Path Planning for a Simulated Vehicle**

The goals / steps of this project are the following:

* Implement a path planning algorithm that allows an autonomous vehicle to safely navigate highway traffic in a simulator.  The planned path must ensure that no collision occurs with other vehicles, and that the ego vehicle obeys constraints on speed, acceleration, and jerk. 

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### 1. Compilation###

**The code compiles corectly.**

- Done.  See compile instructions in the [README.md](README.md) file.

### 2. Valid Trajectories

**The car is able to drive at least 4.32 miles without incident**

**The car drives according to the speed limit**

**Maximum acceleration and jerk are  not exceeded**

**Car does not have collisions**

**The car stays in its lane, except for the time between changing lanes**

**The car is able to change lanes**

- Done.  Please refer to the screenshot below.  At the time of the screenshot, the car had completed 23.45 miles in a time of 31 minutes and 13 seconds.  This equates to an average speed of 45.1 miles per hour.

### 3. Reflection ###

**There is a reflection on how to generate paths**

The path planning algorithm I developed is simple --- there was no need to implement an A* path planning algorithm for example.

I found the following steps helpful for developing the algorithm:

1. Use lane coordinates to ensure that the car can accelerate and drive in its original lane (without concern for other vehicles).  The proposed trajecotry is generated using splines to ensure that dynamic constraints are never violated.
2. Use a "path following" formula to ensure that the vehicle can safely decelerate for cars ahead.  At this point, the car can safely drive around the track in the center lane, but is not able to overtake any cars in front of it.
3.  Build a lane scoring function that scores each lane, and prioiritizes lanes.  Lane score is determined by "exit velocity", which is the speed of the first blockage ahead in the lane.  If the lane is empty,  then the exit velocity is infinite.  At this point, the ego vehicle is able to determine which lane it would like to be in.
4. Build a safety checker to see if it is safe to change lanes using the lane coordinates of all other vehicles.
5. Build a simple finite state machine in which the state is simply the ID of the currently desired lane.  At each timestep, if the car is currently in the desired lane, then each lane is examined in order of priority to determine if it is safe to move to a lane of higher priority.  Then, this higher scoring lane becomes the desired lane.

Putting these parts together provides a path planning algorithm.  The path planning algorithm ensures speed, acceleration, and jerk constraints are satisfied by construction.  Lane changes are only inititated if it is safe to do so and no collisions will occur.

