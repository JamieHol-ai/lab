#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, jointlimitsviolated
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET

from scipy.optimize import fmin_slsqp

def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    '''Return a collision free configuration grasping a cube at a specific location and a success flag'''
    setcubeplacement(robot, cube, cubetarget)

    #for i, n in enumerate(robot.model.names):
    #   print(i, n)

    def offset(eff_position, cube_position):
            handMhook = eff_position.inverse() * cube_position
            vec = pin.log(handMhook).vector
            return vec

    def cost(q):
        pin.framesForwardKinematics(robot.model, robot.data, q)
        left_offset = offset(robot.data.oMf[left_id], left_cube_placement)
        right_offset = offset(robot.data.oMf[right_id], right_cube_placement)

        return norm(left_offset)**2 + norm(right_offset)**2

    qstart = projecttojointlimits(robot, qcurrent)

    left_id = robot.model.getFrameId(LEFT_HAND)
    left_cube_placement = getcubeplacement(cube, LEFT_HOOK)
    right_id = robot.model.getFrameId(RIGHT_HAND)
    right_cube_placement = getcubeplacement(cube, RIGHT_HOOK)

    joint_limits = list(zip(robot.model.lowerPositionLimit, robot.model.upperPositionLimit))

    popt = fmin_slsqp(cost, qstart, bounds=joint_limits)
    print(popt)

    if cost(popt) > 1e-3: #TODO: come up with a proper threshold. 
        print("Cube not graspped")
        return popt, False
    
    if jointlimitsviolated(robot, popt):
        print("Joint limits violated")
        return popt, False

    if collision(robot, popt):
        print("Config results in collision") 
        return popt, False

    print("All good")
    return popt, True
            
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    
    
    
"""
q0 = base rotation
q1 = head
q2 = neck
q3 = left arm shoulder left right
q4 = left arm shoulder up down
q5 = left arm elbow
q6 - left wrist rotation left right
q7 = left wrist rotation up down
q8 = ??
q9 = right arm shoulder left right
q10 = right arm shoulder up down
q11 = right arm elbow
q12 = right wrist rotation left right
q13 = right wrist rotation up down
q14 = ???
"""