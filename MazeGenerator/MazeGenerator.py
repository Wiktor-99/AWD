import sys
import pygame

WIDTH = 900
HEIGHT = 600
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("")


WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Spot:
    def __init__(self, row, col, width):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.width = width

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def isBarrier(self):
        return self.color == BLACK

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def draw(self, win):
        pygame.draw.rect(
            win, self.color, (self.x, self.y, self.width, self.width))

def make_grid(rows, columns, width):
    grid = []
    gap = width // columns
    for i in range(rows):
        grid.append([])
        for j in range(columns):
            spot = Spot(j, i, gap)
            grid[i].append(spot)

    return grid


def draw_grid(win, rows, columns, width, height):
    gap = width // columns
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(columns):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, height))


def draw(win, grid, rows, columns, width, height):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, columns, width, height)

    pygame.display.update()


def get_clicked_pos(pos, rows, columns, width, height):
    gapX = width // columns
    gapY = height // rows
    y, x = pos

    row = y // gapY
    col = x // gapX

    return row, col

def main(win, width, height, args):
    if len(args) < 1:
        print(f'No output file name, run with file_name.txt')
        return
    fileName = args[0]
    COLUMNS = 30
    ROWS = 20
    halfSizeOfBlock = 15
    grid = make_grid(ROWS, COLUMNS, width)
    run = True
    start = None
    end = None
    while run:
        draw(win, grid, ROWS, COLUMNS, width, height)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                col, row = get_clicked_pos(pos, ROWS, COLUMNS, width, height)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                col, row = get_clicked_pos(pos, ROWS, COLUMNS, width, height)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    with open(fileName, 'w') as file:
                        file.write(f'{start.x + halfSizeOfBlock},{start.y + halfSizeOfBlock}\n')
                        file.write(f'{end.x},{end.y}\n')
                        for row in grid:
                            for spot in row:
                                if spot.isBarrier():
                                    file.write(f'{spot.x},{spot.y}\n')

    pygame.quit()

if __name__ == '__main__':
    main(WIN, WIDTH, HEIGHT, sys.argv[1:])