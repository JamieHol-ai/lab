# Other Software Needed
No other software is required. Provided the files are executed in the aro2026 venv, all neccessary files should run as-is.

# Algorithm Descriptions

## Part 1: Geometry
### 1.I
An high-level description of `computegrasppose` is as follows:
 - Validates starting configuration and computes left/right end effector positions
 - Uses the scipy optimiser `fmin-slsqp` to comptute a configuration for the robot that grasps the cube:
   - Cost defined as the squared distances of the end-effectors from their targets 
   - Joint limits passed as bounds
 - Extra checks performed to determine success:
   - Cost of the returned configuration above defined threshold in `config.py`?
   - Joint limits voilated?
   - Is the result in-collision with anything?
   - Is the robot too close to the obstacle?
   - Is the cube too close to the obstacle?
   If any are true, the configuration is considered invalid and `False` is returned

### 1.II
To generate a path, an RRT approach is used:
 - Sample a cube placement within bounds of the problem area
 - Find nearest node in tree to the sample
 - Restrict sampled to within a set distance of the nearest node
 - Compute reachability by checking along the translation vector using a defined step size and `computegrasppose`
   - If not, loop back
   - If yes, this node is added to the tree, and the reachability of the goal is checked. If the goal can be reached, a path can then be computed backwards through the tree.
If the number of steps taken in the RRT algorithm is above a defined value, it is assumed there is no path and `None` is returned. 

## Part 2: Dynamics
### 2.I Path to Trajectory
The smooth/interpolate the path to obtain a trajectory, the Bezier library provided is used to compute a trajectory composed of the smoothed path, plus the first and second derivative of that path at the smoothed points.

### 2.II 


# Extra Points Explanation
 - Level 1 of trajectory optimisation compeleted using Bezier.
 - "Implementing an efficient search for the nearest neighbour in the RRT" completed by implementing a hashgrid rather than looping through every node. (good demo found [here](https://www.redblobgames.com/x/1730-spatial-hash/))