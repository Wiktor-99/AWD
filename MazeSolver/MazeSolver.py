import pygame
import numpy as np
from AppWidgets.Colors import GREEN, BLACK, RED
from AppWidgets.Button import Button
from PathCreateAlgorithms.Voronoi import VornoiPathFinder
import PathCreateAlgorithms.rrt as RRT
from Robot.Robot import Robot
import time as t

def getPointsFromFile():
    points = np.genfromtxt('MazeSolver/assets/maze.txt', delimiter=',', dtype=np.int32)

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

def drawPathPoints(points, window, radius=5):
    for x, y in points:
        pygame.draw.circle(window, RED, (x,y), radius)


def get_font(size):
    return pygame.font.Font("MazeSolver/assets/LeagueSpartan-Bold.otf", size)

def createButtons():
    option_rect = pygame.image.load("MazeSolver/assets/Options Rect.png")
    font = get_font(60)
    voronoi_button = Button(image=option_rect, pos=(450, 200), text_input="Voronoi", font=font,
                             base_color="#b68f40", hovering_color="White")
    rrt_button = Button(image=option_rect, pos=(450, 330), text_input="RRT", font=font,
                        base_color="#b68f40", hovering_color="White")
    result_button = Button(image=option_rect, pos=(450, 460), text_input="Wyniki",
                        font=font, base_color="#b68f40", hovering_color="White")

    return { "voronoi" : voronoi_button, "rrt" : rrt_button, "result" : result_button}

def simulationLoop(robot, path, screen, obstacleList, end):
    run = True
    while run:
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN:
                if event.key == ord ( "m" ):
                    pygame.display.set_caption("Menu")
                    run = False

        screen.fill(BLACK)
        drawPathPoints(path, screen)
        drawObstacles(obstacleList, screen)
        robot.moveRobot(path)
        robot.draw(screen)
        drawCircleOnEndOfPath(screen, end)
        pygame.display.update()

def countPathLength(path):
    total_length = 0
    for i in range(len(path)):
        if i + 1 >= len(path):
            break
        total_length += ((path[i+1][0] - path[i][0]) ** 2 + (path[i+1][1] - path[i][1])**2)**(1/2)

    return total_length

def createResultText(screen, results, algorithm_name, text_center=(80, 150), length_center=(340, 200), time_center=(340, 150),
                      time_rect_center=(600, 150), path_rect = (600, 200)):
    text = get_font(45).render(f"{algorithm_name}", True, "#d7fcd4")
    rect = text.get_rect(center=text_center)
    length_text = get_font(35).render(str(results[f'{algorithm_name}_path_length']), True, "#d7fcd4")
    length_rect = length_text.get_rect(center=length_center)
    time_text = get_font(35).render(str(results[f'{algorithm_name}_time']), True, "#d7fcd4")
    time_rect = time_text.get_rect(center=time_center)
    sekund = get_font(45).render("sekund", True, "#d7fcd4")
    sekund_rect = sekund.get_rect(center=time_rect_center)
    path = get_font(45).render("jednostek", True, "#d7fcd4")
    path_rect = path.get_rect(center=path_rect)


    screen.blit(text, rect)
    screen.blit(sekund, sekund_rect)
    screen.blit(length_text, length_rect)
    screen.blit(time_text, time_rect)
    screen.blit(path, path_rect)

def resultBackground(screen):
    screen.blit(pygame.image.load("MazeSolver/assets/Background.png"), (0, 0))
    results_text = get_font(45).render("Wyniki ostatnich przejazdów", True, "#d7fcd4")
    results_rect = results_text.get_rect(center=(450, 50))
    screen.blit(results_text, results_rect)

def createResultBackButton(screen):
    results_back = Button(image=None, pos=(450, 500),
            text_input="Powrót", font=get_font(65), base_color="#b68f40", hovering_color="Green")

    results_back.changeColor(pygame.mouse.get_pos())
    results_back.update(screen)

    return results_back

def drawCircleOnEndOfPath(screen, end):
    pygame.draw.circle(screen, RED, end, 20, 1)

def checkResults(screen, results):
    while True:
        resultBackground(screen)
        createResultText(screen, results, 'rrt')
        createResultText(screen, results, 'voronoi', (130, 300), (340, 350), (340, 300), (600, 300), (600, 350))
        results_back = createResultBackButton(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if results_back.checkForInput(pygame.mouse.get_pos()):
                    main()

        pygame.display.update()


def main():
    pygame.init()
    points, start, end = getPointsFromFile()
    windowSize = (900,600)
    screen = pygame.display.set_mode(windowSize)
    obstacleList = createObstacleList(points)
    robotRadius = round(10 * 2**(0.5))
    pygame.display.set_caption("Menu")

    back_ground = pygame.image.load("MazeSolver/assets/Background.png")
    menu_text = get_font(55).render("Wybór algorytmu", True, "#b68f40")
    menu_rect = menu_text.get_rect(center=(450, 60))
    buttons = createButtons()
    results = {'voronoi_path_length': 0, 'voronoi_time' : 0, 'rrt_path_length': 0, 'rrt_time' : 0}

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
                robot = Robot((start[0], start[1]), robotRadius)
                if buttons["voronoi"].checkForInput(menu_mouse_pos):
                    pygame.display.set_caption("Voronoi")
                    screen.fill(BLACK)
                    drawObstacles(obstacleList, screen)
                    pygame.display.update()
                    start_time = t.time()
                    voronoi = VornoiPathFinder(start, end, points)
                    path = voronoi.findVoronoiPath()
                    end_time = t.time()
                    results['voronoi_path_length'] = round(countPathLength(path), 2)
                    results['voronoi_time'] = round(end_time - start_time, 4)
                    simulationLoop(robot, path, screen, obstacleList, end)


                if buttons["rrt"].checkForInput(menu_mouse_pos):
                    pygame.display.set_caption("RRT")
                    screen.fill(BLACK)
                    drawObstacles(obstacleList, screen)
                    pygame.display.update()
                    start_time = t.time()
                    rrt = RRT.RRT(windowSize, start, end,obstacleList, points,screen)
                    path = RRT.findRRTPath(rrt)
                    end_time = t.time()
                    results['rrt_path_length'] = round(countPathLength(path), 2)
                    results['rrt_time'] = round(end_time - start_time, 4)
                    simulationLoop(robot, path, screen, obstacleList, end)

                if buttons["result"].checkForInput(menu_mouse_pos):
                    checkResults(screen, results)

            pygame.display.update()


if __name__ == '__main__':
    main()