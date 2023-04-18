import pygame
import numpy as np


def getPointsFromFile():
    points = np.genfromtxt('maze.txt', delimiter=',', dtype=np.int32)

    start = points[0]
    end = points[1]
    return points[2:], start, end

def main():
    pygame.init()
    points, start, end = getPointsFromFile()

    screen = pygame.display.set_mode((900, 720))

    while True:


        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            pygame.display.update()


if __name__ == '__main__':
    main()