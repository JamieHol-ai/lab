# Other Software Needed
No other software is required. Provided the files are executed in the aro2026 venv, all neccessary files should run as-is.

# Algorithm Descriptions

## Part 1: Geometry
### 1.I
An high-level description of `computegrasppose` is as follows:
 - Clamps given starting configuration within joint limits
 - Computes left and right end effector position
 - Uses the scipy optimiser `fmin-slsqp` to comptute a configuration for the robot that grasps the cube:
   - Cost function is defined as $$|\text{left\_offset}|^2 + |\text{right\_offset}|^2$$
     where the left and right offsets are the vectors to the desired target
   - Joint limits are passed to the optimiser as bounds
 - Finally, some extra checks are performed to determine the success of the function:
   - Is the cost of the returned configuration above the threshold defined in `config.py`?
   - Are joint limits voilated?
   - Is the result in-collision with anything?
   - Does the configuration result with the robot too close to the obstacle?
   - Does the configuration result with the cube being too close to the obstacle?
   If any of the above are true, the configuration is considered invalid and `False` is returned along with that configuration

### 1.II
To generate a path, an RRT approach is used:
 - Sample a cube placement within set bounds of the problem area
 - Find nearest node in tree to the sampled placement
 - Restrict sampled placement to within a set distance of the nearest node
 - Determine whether this placement can be reached by checking along the translation vector using a defined step size `computegrasppose`
   - If not, this placement is discarded and the loop begins again
   - If yes, this node is added to the tree, and the reachability of the goal is checked. If the goal can be reached, a path can then be computed backwards through the tree.
If the number of steps taken in the RRT algorithm is above a defined value, it is assumed a path may not be found and one is not returned. 

## Part 2: Dynamics
### 2.I Path to Trajectory
The smooth/interpolate the path to obtain a trajectory, the Bezier library provided is used to compute a trajectory composed of the smoothed path, plus the first and second derivative of that path at the smoothed points.

### 2.II 

# Extra Points Explanation