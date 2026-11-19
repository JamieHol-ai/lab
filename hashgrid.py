from typing import Any
from math import floor, ceil
from numpy.linalg import norm
from numpy import argmin

Node = tuple[None | int, Any, list[Any]]

class Hashgrid:
    """
    Stores nodes in both a list and a parallel structure that allows
    for fast retrival of nearest neighbour in 3d.

    Node structure:
    (indexOfNode, configuration_q, listOfConnectedNodes)
    """

    def __init__(self, initialNode: Node, quadsize: float = 1):
        self.tree: list[Node] = []
        self.quadsize: float = quadsize
        self.quads: dict[tuple[int, int, int], list[int]] = {}
        self.total_norms = 0
        self.append(initialNode)

    def append(self, node: Node):
        """
        Add new node to the quadtree
        """
        self.tree.append(node)
        quadPos = self.__getQuadPos(node[1].translation)
        if self.quads.get(quadPos):
            self.quads[quadPos].append(len(self.tree)-1)
        else:
            self.quads[quadPos] = [len(self.tree)-1]

    def __getitem__(self, index) -> Node:
        return self.tree[index]

    def __len__(self):
        return len(self.tree)

    def getNearestNodeTo(self, q) -> int:
        """
        Returns index of node nearest to given config q
        """
        if len(self.tree) == 0:
            return 0

        if len(self.tree) < 10:
            return self.__getClosestNodeIndex(q.translation, list(range(0, len(self.tree))))[0]

        quadPos: tuple[int, int, int] = self.__getQuadPos(q.translation)

        dist = 0
        closestNodeIndex = -1
        closestDist = -1
        while True:
            nodes: list[int] = self.__getXQuadDistanceNodes(quadPos, dist)
            if len(nodes) != 0:
                closestNodeIndex, closestDist = self.__getClosestNodeIndex(q.translation, nodes)
                break
            dist += 1

        for r in range(dist + 1, ceil(closestDist / self.quadsize) + 1):
            nodes: list[int] = self.__getXQuadDistanceNodes(quadPos, r)
            if len(nodes) != 0:
                maybeClosestNodeIndex, maybeClosestDist = self.__getClosestNodeIndex(q.translation, nodes)
                if maybeClosestDist < closestDist:
                    closestDist = maybeClosestDist
                    closestNodeIndex = maybeClosestNodeIndex

        return closestNodeIndex

    def __getQuadPos(self, translation) -> tuple[int, int, int]:
        return (floor(translation[0] / self.quadsize), floor(translation[1] / self.quadsize), floor(translation[2] / self.quadsize))

    def __getXQuadDistanceNodes(self, quadPos: tuple[int, int, int], dist: int) -> list[int]:
        """
        Return all nodes from quads a given distance away from the starting quad
        """
        if dist == 0:
            return self.quads.get(quadPos, [])

        output = []
        full = range(-dist, dist + 1)
        inner = range(-dist + 1, dist)

        for dx in (-dist, dist):
            for dy in full:
                for dz in full:
                    output.extend(self.quads.get((quadPos[0] + dx, quadPos[1] + dy, quadPos[2] + dz), []))

        for dy in (-dist, dist):
            for dz in full:
                for dx in inner:
                    output.extend(self.quads.get((quadPos[0] + dx, quadPos[1] + dy, quadPos[2] + dz), []))

        for dz in (-dist, dist):
            for dx in inner:
                for dy in inner:
                    output.extend(self.quads.get((quadPos[0] + dx, quadPos[1] + dy, quadPos[2] + dz), []))

        return output

    def __getClosestNodeIndex(self, translation, nodes: list[int]) -> tuple[int, float]:
        if len(nodes) == 0:
            return -1, -1
        
        dists = []
        for index in nodes:
            dists.append(norm(self.tree[index][1].translation - translation))
            self.total_norms += 1
        lowestDistIndex = int(argmin(dists))
        index = nodes[lowestDistIndex]
        return index, dists[lowestDistIndex]