Should run as is in the aro2026 venv.
# Algorithm Descriptions

## Part 1: Geometry
### 1.I
`computeqgrasppose`:
 - Validates the starting configuration and computes the end effector positions
 - Uses scipy's `fmin_slsqp` to find a grasping configuration:
   - Cost is the squared distance of each end effector from its target
   - Joint limits passed as bounds
 - The result is invalid (`False` returned) if:
   - Cost is above the threshold in `config.py`
   - Joint limits are violated
   - It is in collision
   - The robot or cube is too close to the obstacle

### 1.II
Paths are generated with RRT:
 - Sample a cube placement within the problem bounds
 - Find the nearest tree node and restrict the sample to within a set distance of it
 - Check reachability along the translation vector in fixed steps using `computeqgrasppose`
   - If unreachable, resample
   - If reachable, add the node to the tree and check whether the goal is reachable; if so, the path is traced back through the tree
If the step limit is exceeded, `None` is returned.

 - Robustness: RRT sampling bounds are computed at run time from the start, goal and obstacle (plus margin)

## Part 2: Dynamics
### 2.I Path to Trajectory
The provided Bezier library smooths the path into a trajectory, along with its first and second derivatives.

### 2.II Control Law
An inverse dynamics controller tracks the trajectory using PD feedback, and an additional inward force applied at each end effector via $J^T W$ keeps the cube gripped.


# Extra Points Explanation
 - Level 1 of trajectory optimisation completed using Bezier.
 - "Implementing an efficient search for the nearest neighbour in the RRT" completed by using a hashgrid rather than looping through every node (good demo [here](https://www.redblobgames.com/x/1730-spatial-hash/)).
