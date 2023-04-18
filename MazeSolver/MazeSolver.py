import pygame
import numpy as np
from AppWidgets.Colors import GREEN

def getPointsFromFile():
    points = np.genfromtxt('maze.txt', delimiter=',', dtype=np.int32)

    start = points[0]
    end = points[1]
    return points[2:], start, end

def createObstacleList(points):
    obstacleSize = 30
    obstacleList = []
    for x, y in points:
        rect = pygame.Rect((x,y),(obstacleSize, obstacleSize))
        obstacleList.append(rect)

    return obstacleList

def drawObstacles(obstacleList, window):
    for rect in obstacleList:
        pygame.draw.rect(window,GREEN,rect)

def main():
    pygame.init()
    points, _, _ = getPointsFromFile()
    windowSize = (900,600)
    screen = pygame.display.set_mode(windowSize)
    obstacleList = createObstacleList(points)


    while True:
        drawObstacles(obstacleList, screen)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            pygame.display.update()


if __name__ == '__main__':
    main()