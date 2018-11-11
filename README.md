# Path Planning

![Path Planning](score.png)

This project is in conjunction with the Udacity Self-Driving Car course.  In particular, the goal of this project is to implement a path planning algorithm that allows an autonomous vehicle to safely navigate highway traffic in a simulator.

* More detail about the approach is provided in [writeup.md](./writeup.md).

## Getting Started

### Prerequisites

```
CMake
uWS		(Socket for interfacing with Udacity simulator)	
```

### Installing

1. Clone this project from GitHub:

```
git clone https://github.com/jimwatt/P11-pathplanning.git
```

2. Change into the `build` directory, and compile with `CMake`.

   ```
   cd build
   cmake ..
   make
   ```

## Running the Code

* Run the executable from the `build` directory:
  ```./path_planning```
* The executable will wait until sensor fusion telemetry data are provided by the simulator through the uWS socket.





## Authors

* **James Watt**

<!--## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details-->

## Acknowledgments
This project is a submission to the Udacity Self-Driving Car nanodegree:

* ```
  https://github.com/udacity/CarND-MPC-Control-Project.git
  ```

## Path Planning

------

**Path Planning for a Simulated Vehicle**

The goals / steps of this project are the following:

- Implement a path planning algorithm that allows an autonomous vehicle to safely navigate highway traffic in a simulator.  The planned path must ensure that no collision occurs with other vehicles, and that the ego vehicle obeys constraints on speed, acceleration, and jerk. 

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points

### 1. Compilation

**The code compiles corectly.**

- Done.  See compile instructions in the [README.md](README.md) file.

### 2. Valid Trajectories

**The car is able to drive at least 4.32 miles without incident**

**The car drives according to the speed limit**

**Maximum acceleration and jerk are  not exceeded**

**Car does not have collisions**

**The car stays in its lane, except for the time between changing lanes**

**The car is able to change lanes**

- Done.  Please refer to the screenshot below.  At the time of the screenshot, the car had completed 23.45 miles in a time of 31 minutes and 13 seconds with no collisions or constraint violations.  This equates to an average speed of 45.1 miles per hour.

![Path PLanning](score.png)

### 3. Reflection

**There is a reflection on how to generate paths**

The path planning algorithm I developed is simple --- there was no need to implement an A* path planning algorithm, for example.

I found the following steps helpful for developing the algorithm:

1. **Spline Trajectories:**  Use lane coordinates to ensure that the car can accelerate and drive in its original lane (without concern for other vehicles), while obeying constraints on speed, acceleration, and jerk.  The proposed trajectory is generated using splines to ensure that dynamic constraints are never violated.  The spline path planner can easily be modified to generate trajectories to adjacent lanes.
2. **Car Following:**  Use a "car following" formula to ensure that the vehicle can safely decelerate for slower cars ahead in the same lane.  At this point, the car can safely drive around the track in the center lane, but is not able to safely maneuver to overtake any cars in front of it.
3. **Lane Scoring:**  Build a lane scoring function that scores each lane, and prioritizes lanes.  Lane score is determined by "exit velocity", which is the current speed of the first blockage ahead in the lane.  If the lane is empty,  then the exit velocity is infinite.  At this point, the ego vehicle is able to determine which lane it would like to be in to improve average velocity around the track.
4. **Collision Checker:**  Build a safety checker to see if it is safe to change lanes.  To ensure that no collisions occur, the (future) lane coordinates of all other vehicles must be examined.
5. **Finite State Machine:**  Build a simple finite state machine in which the state is simply the ID of the currently desired lane.  At each timestep, if the car is currently in the desired lane, then each lane is examined in order of priority to determine if it is safe to move to a lane of higher priority.  Then, this higher scoring lane becomes the new desired lane.  The final step is to plan a path toward the desired lane --- if the current lane is the desired lane, then the proposed path will be directly forward in the lane.  
   One additional line of logic allows the car to move safely toward a faster lane, even if the desired faster lane is not immediately adjacent.

Putting these parts together provides a path planning algorithm.  The path planning algorithm ensures speed, acceleration, and jerk constraints are satisfied by construction.  Lane changes are only inititated if it is safe to do so and no collisions will occur.

