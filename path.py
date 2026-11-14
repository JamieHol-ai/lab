#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import norm

from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, LEFT_HAND, LEFT_HOOK
from inverse_geometry import computeqgrasppose
from setup_pinocchio import setuppinocchio
from tools import setcubeplacement
import time
import math

position_bounds = {"upper": [0.50, 0.20, 1.2, 0., 0., 0.],
                   "lower": [0.20, -0.40, 0.8, 0., 0., 0.]}

robot, cube = None, None

def setupmodels():
    global robot, cube
    if robot is None:
        robot, _, _, cube = setuppinocchio()

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
    setupmodels()
    k = 5000 # max number of samples
    delta = 0.1 # max distance the cube moves per tree extension
    step_size = 0.01 # marching discretisation
    tree = [(None, cubeplacementq0, [qinit])]

    for step in range(k):
        placement_rand = sample()
        near_index = nearest_vertex(tree, placement_rand)
        parent, placement_near, qs_near = tree[near_index]
        placement_rand = limit_step(placement_near, placement_rand, delta)
        qs, placement_new, _ = marching(qs_near[-1], placement_near, placement_rand, step_size)
        if len(qs) < 2:
            continue # no progress made
        tree.append((near_index, placement_new, qs))
        qs_goal, _, success = marching(qs[-1], placement_new, cubeplacementqgoal, step_size)
        if success:
            print("Path found!")
            tree.append((len(tree) - 1, cubeplacementqgoal, qs_goal))
            return getpath(tree)
    print("path not found")
    return None

def nearest_vertex(tree, placement):
    # returns the index of the closest node to placement
    dists = []
    for node in tree:
        dists.append(norm(node[1].translation - placement.translation))
    return int(np.argmin(dists))

def limit_step(placement_near, placement_rand, delta):
    # limits a step to delta distance
    direction = placement_rand.translation - placement_near.translation
    dist = norm(direction)
    if dist <= delta:
        return placement_rand
    return pin.SE3(placement_rand.rotation, placement_near.translation + direction * delta / dist)

def getpath(tree):
    # returns a path (configuration list) from a tree
    segments = []
    node = tree[-1]
    while node[0] is not None:
        segments = [node[2][1:]] + segments # first q of a segment is the parent's last q
        node = tree[node[0]]
    path = [node[2][0]]
    for segment in segments:
        path += segment
    return path

def sample():
    target = np.random.uniform(low=position_bounds["lower"], high=position_bounds["upper"])
    target_placement = pin.SE3(pin.rpy.rpyToMatrix(target[3:]), np.array(target[:3]))
    return target_placement

def marching(qinit, init_placement, final_placement, step_size):
    direction = final_placement.translation - init_placement.translation
    length = norm(direction)
    steps = math.ceil(length/step_size) + 1
    points = np.linspace(init_placement.translation, final_placement.translation, steps)
    qs = [qinit]
    success = True
    q = qinit
    reached = init_placement
    for point in points[1:]:
        target = pin.SE3(init_placement.rotation, point)
        q, valid = computeqgrasppose(robot,q,cube,target)
        if not valid:
            success = False
            break
        qs.append(q)
        reached = target
    return qs, reached, success
    



def displaypath(robot,path,dt,viz):
    cubeMhook = cube.data.oMf[cube.model.getFrameId(LEFT_HOOK)]
    left_id = robot.model.getFrameId(LEFT_HAND)
    for q in path:
        pin.framesForwardKinematics(robot.model, robot.data, q)
        oMcube = robot.data.oMf[left_id] * cubeMhook.inverse()
        setcubeplacement(robot, cube, oMcube)
        viz.display(q)
        time.sleep(dt)


if __name__ == "__main__":
    from tools import setupwithmeshcat
    robot, cube, viz = setupwithmeshcat()

    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)

    if not(successinit and successend):
        print ("error: invalid initial or end configuration")

    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    if path is not None:
        displaypath(robot,path,dt=0.05,viz=viz)
    
