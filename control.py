#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np
import pinocchio as pin
from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 300.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    position_error = trajs[0](tcurrent) - q
    velocity_error = trajs[1](tcurrent) - vq

    acceleration = trajs[2](tcurrent) + Kp*position_error + Kv*velocity_error

    M = pin.crba(robot.model, robot.data, q)
    h = pin.nle(robot.model, robot.data, q, vq)

    W_left = np.array([0,-50,0,0,0,0])
    left_id = robot.model.getFrameId("LARM_EFF")
    torque_left = pin.computeFrameJacobian(robot.model, robot.data, q, left_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).transpose() @ W_left

    W_right = np.array([0,50,0,0,0,0])
    right_id = robot.model.getFrameId("RARM_EFF")
    torque_right = pin.computeFrameJacobian(robot.model, robot.data, q, right_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).transpose() @ W_right
    
    torques = M @ acceleration + h + torque_left + torque_right

    sim.step(torques)

if __name__ == "__main__":
        
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
    #setting initial configuration
    sim.setqsim(q0)
    
    
    #TODO this is just an example, you are free to do as you please.
    #In any case this trajectory does not follow the path 
    #0 init and end velocities
    def maketraj(path,T):
        qs = [path[0]] + path + [path[-1]]
        q_of_t = Bezier(qs,t_max=T)
        vq_of_t = q_of_t.derivative(1)
        vvq_of_t = vq_of_t.derivative(1)
        return q_of_t, vq_of_t, vvq_of_t
    
    
    total_time=8.
    trajs = maketraj(path, total_time)   
    
    tcur = 0.
    
    
    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
    
    
    