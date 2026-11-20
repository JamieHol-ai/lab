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


# Extra Points Explanation