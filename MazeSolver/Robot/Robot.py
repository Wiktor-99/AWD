
import pygame
import math
import numpy as np


class Robot:
    def __init__(self, startPos, robotWidth):
        self.meters2pixels = 3779.52
        self.x = startPos[0]
        self.y = startPos[1]
        self.robotWidth = robotWidth
        self.theta = 0
        self.robotRotVel = 0.01*self.meters2pixels
        self.robotLinVel = 0.01*self.meters2pixels
        self.a = 6
        self.waypoint = 0
        self.robotImg = pygame.image.load("MazeSolver/assets/AGV.png")
        self.rotatedImg = self.robotImg
        self.rect = self.rotatedImg.get_rect(center = (self.x,self.y))
        self.last_x = self.x
        self.last_y = self.y

    def draw(self, window):
        window.blit(self.rotatedImg, self.rect)

    def nodeDistance(self,pos1,pos2):
        return np.sqrt(np.power(float(pos1[0] - pos2[0]),2) + np.power(float(pos1[1] - pos2[1]),2))

    def moveInverseRobot(self, finalPath):
        nextPos = finalPath[self.waypoint]
        deltaX = (nextPos[0] - self.last_x)
        deltaY = (nextPos[1] - self.last_y)

        self.robotLinVel = deltaX * math.cos(self.theta) + deltaY * math.sin(self.theta)
        self.robotRotVel = (-1/self.a)*math.sin(self.theta) * deltaX + (1/self.a)*math.cos(self.theta) * deltaY

        if self.nodeDistance((self.x,self.y), finalPath[self.waypoint]) < 1:
            self.last_x = nextPos[0]
            self.last_y = nextPos[1]
            self.waypoint +=1

        if self.waypoint > (len(finalPath)-1):
            self.waypoint = len(finalPath)-1

    def moveRobot(self, finalPath):
        dt = self.adjustDt()

        self.x += (self.robotLinVel * math.cos(self.theta) - self.a * math.sin(self.theta) * self.robotRotVel) * dt
        self.y += (self.robotLinVel * math.sin(self.theta) + self.a * math.cos(self.theta) * self.robotRotVel) * dt

        self.theta += self.robotRotVel * dt
        self.rotatedImg = pygame.transform.rotate(self.robotImg , math.degrees(-self.theta))
        self.rect = self.rotatedImg.get_rect(center = (self.x,self.y))
        self.moveInverseRobot(finalPath)

    def adjustDt(self):
        dt = 0.002
        if self.robotLinVel < 30:
            dt = 0.05
        if self.robotLinVel < 10:
            dt = 0.009
        return dt