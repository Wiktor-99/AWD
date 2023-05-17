import numpy as np
import math
import random
import pygame
import colorspy as color


RED = color.red
GREEN = color.green
BLUE = color.blue
YELLOW = color.yellow
WHITE = color.white
BLACK = color.black
PURPLE = color.purple
ORANGE = color.orange
GREY = color.gray
TURQUOISE = color.turquoise

class RRT:
    def __init__(self,windowSize, startPos, endPos, obstaclesList, points, window) -> None:
        self.windowSize = windowSize
        self.winWidth = windowSize[0]
        self.winHight = windowSize[1]
        self.startPos = startPos
        self.endPos = endPos
        self.isFinished = False
        self.graphPoints = []
        self.parents = []
        self.finalPosition = None
        self.path = []
        self.graphPoints.append(self.startPos)
        self.parents.append(0)
        self.points = points
        self.WINDOW = window
        self.obstaclesList = obstaclesList


    def nodeAdd(self, id, position):
        self.graphPoints.insert(id,position)

    def nodeDelete(self,id):
        self.graphPoints.pop(id)

    def nodeDistance(self, id1, id2):
        X = float(self.graphPoints[id1][0] - self.graphPoints[id2][0])
        Y = float(self.graphPoints[id1][1] - self.graphPoints[id2][1])

        return np.sqrt(np.power(X,2)+np.power(Y,2))

    def edgeAdd(self, parentId, childId):
        self.parents.insert(childId,parentId)

    def edgeRemove(self, id):
        self.parents.pop(id)

    def randomDirectionPoint(self):
        x = int(random.uniform(0, self.winWidth))
        y = int(random.uniform(0, self.winHight))

        return x,y

    def nodeCollisionDetection(self):
        id = len(self.graphPoints) - 1

        for rec in self.obstaclesList:
            if rec.collidepoint(self.graphPoints[id]):
                self.nodeDelete(id)
                return False

        return True

    def edgeCollisionDetection(self, nodeParent, nodeChild):

        for rec in self.obstaclesList:
            for i in range(0,101):
                u = i/100
                x = nodeParent[0]*u + nodeChild[0]*(1-u)
                y = nodeParent[1]*u + nodeChild[1]*(1-u)

                if rec.collidepoint((x,y)):
                    return False
        return True

    def nodeConnection(self, parentId, childId):
        if self.edgeCollisionDetection(self.graphPoints[parentId], self.graphPoints[childId]):
            self.edgeAdd(parentId,childId)
            return True

        else:
            self.nodeDelete(childId)
            return False

    def measureNodeDistance(self,nodeId):
        distanceMin = self.nodeDistance(0,nodeId)
        nearestId = 0
        for i in range(0,nodeId):
            if self.nodeDistance(i,nodeId) < distanceMin:
                nearestId = i
                distanceMin = self.nodeDistance(i,nodeId)
        return nearestId

    def stepMove(self,nodeNearestId,nodeRandomId,stepSize = 8)->None:
        distance = self.nodeDistance(nodeNearestId,nodeRandomId)

        if distance > stepSize:

            nodeNearestPosition = (self.graphPoints[nodeNearestId][0],self.graphPoints[nodeNearestId][1])
            nodeRandomPosition = (self.graphPoints[nodeRandomId][0],self.graphPoints[nodeRandomId][1])

            diffX = nodeRandomPosition[0] - nodeNearestPosition[0]
            diffY = nodeRandomPosition[1] - nodeNearestPosition[1]

            theta = math.atan2(diffY, diffX)

            nodePosition = (int(nodeNearestPosition[0] + stepSize*math.cos(theta)),
                            int(nodeNearestPosition[1] + stepSize*math.sin(theta)))
            self.nodeDelete(nodeRandomId)

            if abs(nodePosition[0] - self.endPos[0])<stepSize and abs(nodeNearestPosition[1] - self.endPos[1]) <= 20:
                self.nodeAdd(nodeRandomId,self.endPos)
                self.finalPosition = nodeRandomId
                self.isFinished = True

            else:
                self.nodeAdd(nodeRandomId,nodePosition)

    def moveToEndPos(self,nodeEndPosition):
        tempId = len(self.graphPoints)

        self.nodeAdd(tempId,nodeEndPosition)

        nearestToEndPos = self.measureNodeDistance(tempId)
        self.stepMove(nearestToEndPos,tempId)
        self.nodeConnection(nearestToEndPos,tempId)

    def expandTree(self):
        tempId = len(self.graphPoints)

        randomNodePosition = self.randomDirectionPoint()
        self.nodeAdd(tempId,randomNodePosition)

        if self.nodeCollisionDetection():
            nearestNode = self.measureNodeDistance(tempId)
            self.stepMove(nearestNode,tempId)
            self.nodeConnection(nearestNode,tempId)

    def drawFinalPath(self):
        positionId = len(self.graphPoints) -1
        while positionId > 0:
            pygame.draw.circle(self.WINDOW, RED, self.graphPoints[positionId], 2, 0)
            self.path.append(self.graphPoints[positionId])
            tempPosition = positionId
            positionId = self.parents[positionId]
            pygame.draw.line(self.WINDOW, RED, self.graphPoints[tempPosition], self.graphPoints[positionId], 2)
            pygame.display.update()

        pygame.draw.circle(self.WINDOW, RED, self.startPos, 2, 0)
        self.path.append(self.graphPoints[0])
        self.path.reverse()
        pygame.display.update()

        return True


def findRRTPath(rrt):
    iteration = 0
    while (rrt.isFinished == False):
        if iteration % 10 == 0 and rrt.isFinished == False:
            rrt.moveToEndPos(rrt.endPos)
            pygame.draw.circle(rrt.WINDOW, BLUE, rrt.graphPoints[-1], 2, 0)
            pygame.draw.line(rrt.WINDOW, PURPLE, rrt.graphPoints[-1], rrt.graphPoints[rrt.parents[-1]], 2)
            pygame.display.update()

        else:
            rrt.expandTree()
            pygame.draw.circle(rrt.WINDOW, BLUE, rrt.graphPoints[-1], 2, 0)
            pygame.draw.line(rrt.WINDOW, PURPLE, rrt.graphPoints[-1], rrt.graphPoints[rrt.parents[-1]], 2)

        iteration+=1
    rrt.drawFinalPath()
    path = rrt.path
    return path

def countPathLength(path):
        total_length = 0
        for i in range(len(path)):
            if i + 1 >= len(path):
                break
            total_length += ((path[i+1][0] - path[i][0]) **
                            2 + (path[i+1][1] - path[i][1])**2)**(1/2)

        return total_length
