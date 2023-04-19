import pygame
import numpy as np
from AppWidgets.Colors import GREEN, BLACK
from AppWidgets.Button import Button

def getPointsFromFile():
    points = np.genfromtxt('assets/maze.txt', delimiter=',', dtype=np.int32)

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

def get_font(size):
    return pygame.font.Font("assets/LeagueSpartan-Bold.otf", size)

def createButtons():
    option_rect = pygame.image.load("assets/Options Rect.png")
    font = get_font(60)
    voronoi_button = Button(image=option_rect, pos=(450, 200), text_input="Voronoi", font=font,
                             base_color="#d7fcd4", hovering_color="White")
    rrt_button = Button(image=option_rect, pos=(450, 330), text_input="RRT", font=font,
                        base_color="#d7fcd4", hovering_color="White")

    return { "voronoi" : voronoi_button, "rrt" : rrt_button}

def simulationLoop():
    run = True
    while run:
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.event.clear()
                pygame.event.wait()
            if event.type == pygame.KEYDOWN:
                if event.key == ord ( "m" ):
                    run = False


def main():
    pygame.init()
    points, _, _ = getPointsFromFile()
    windowSize = (900,600)
    screen = pygame.display.set_mode(windowSize)
    obstacleList = createObstacleList(points)
    pygame.display.set_caption("Menu")

    back_ground = pygame.image.load("assets/Background.png")
    menu_text = get_font(55).render("Wybór algorytmu", True, "#b68f40")
    menu_rect = menu_text.get_rect(center=(450, 60))
    buttons = createButtons()


    while True:
        screen.blit(back_ground, (0,0))
        screen.blit(menu_text, menu_rect)

        menu_mouse_pos = pygame.mouse.get_pos()

        for _, button in buttons.items():
            button.changeColor(menu_mouse_pos)
            button.update(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.MOUSEBUTTONDOWN:

                if buttons["voronoi"].checkForInput(menu_mouse_pos):
                    screen.fill(BLACK)
                    drawObstacles(obstacleList, screen)
                    simulationLoop()


            pygame.display.update()


if __name__ == '__main__':
    main()