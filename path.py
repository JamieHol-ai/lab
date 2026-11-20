#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import norm

from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, OBSTACLE_PLACEMENT, LEFT_HAND, LEFT_HOOK
from inverse_geometry import computeqgrasppose
from setup_pinocchio import setuppinocchio
from tools import setcubeplacement
import time
import math
from typing import Any
from hashgrid import Hashgrid

BOUNDS_MARGIN = 0.1

robot, cube = None, None

DEBUG = False

def setupmodels():
    global robot, cube
    if robot is None:
        robot, _, _, cube = setuppinocchio()

#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal, quadsize = 0.1):
    setupmodels()
    k = 5000 # max number of samples
    delta = 0.1 # max distance the cube moves per tree extension
    step_size = 0.01 # marching discretisation
    hashgrid: Hashgrid = Hashgrid((None, cubeplacementq0, [qinit]), quadsize)
    bounds = computebounds(cubeplacementq0, cubeplacementqgoal)

    for step in range(k):
        placement_rand = sample(bounds)
        # near_index = nearest_vertex(quadtree.tree, placement_rand)
        near_index = hashgrid.getNearestNodeTo(placement_rand)
        parent, placement_near, qs_near = hashgrid[near_index]
        placement_rand = limit_step(placement_near, placement_rand, delta)
        qs, placement_new, _ = marching(qs_near[-1], placement_near, placement_rand, step_size)
        if len(qs) < 2:
            continue # no progress made
        hashgrid.append((near_index, placement_new, qs))
        qs_goal, _, success = marching(qs[-1], placement_new, cubeplacementqgoal, step_size)
        if success:
            if DEBUG: print("Path found!")
            hashgrid.append((len(hashgrid) - 1, cubeplacementqgoal, qs_goal))
            print(f"Total norms: {hashgrid.total_norms}, number of steps: {step}")
            return getpath(hashgrid.tree)
    if DEBUG: print("Path not found")
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

def computebounds(cubestart, cubegoal):
    obstacle_halfsize = robot.collision_model.geometryObjects[robot.collision_model.getGeometryId('obstaclebase_0')].geometry.halfSide
    obstacle_min = OBSTACLE_PLACEMENT.translation - obstacle_halfsize
    obstacle_max = OBSTACLE_PLACEMENT.translation + obstacle_halfsize
    # hooks' distance from the centre is the cube's half-size
    cube_halfheight = norm(cube.data.oMf[cube.model.getFrameId(LEFT_HOOK)].translation)
    points = np.array([cubestart.translation, cubegoal.translation, obstacle_min, obstacle_max])

    lower = np.min(points, axis=0) - BOUNDS_MARGIN
    upper = np.max(points, axis=0) + BOUNDS_MARGIN
    lower[2] = min(cubestart.translation[2], cubegoal.translation[2])
    upper[2] = max(cubestart.translation[2], cubegoal.translation[2],
                   obstacle_max[2] + cube_halfheight) + BOUNDS_MARGIN
    return {"lower": np.concatenate([lower, np.zeros(3)]),
            "upper": np.concatenate([upper, np.zeros(3)])}

def sample(bounds):
    target = np.random.uniform(low=bounds["lower"], high=bounds["upper"])
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
    robot, cube, viz = setupwithmeshcat(None) # None passed here to start server automatically

    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)

    if not(successinit and successend):
        print ("error: invalid initial or end configuration")

    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    if path is not None:
        displaypath(robot,path,dt=0.05,viz=viz)
    
