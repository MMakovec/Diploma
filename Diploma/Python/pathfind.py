import pygame
import math
import numpy as np
from queue import PriorityQueue

WIDTH = 1920
HEIGHT = 1080

RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,255,0)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)
PURPLE = (128,0,128)
ORANGE = (255,165,0)
GREY = (128,128,128)
TURQUOISE = (64,224,208)


class Spot:
    def __init__(self, row, col, width, height, total_rows, total_cols):
        self.row = row
        self.col = col
        self.x = row*width     # tu zna bit kaj zamešano
        self.y = col*height
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.height = height
        self.total_rows = total_rows
        self.total_cols = total_cols
 
    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE
    
    def make_closed(self):
        self.color = RED
    
    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.height))

    def update_neighbors(self, grid):
        self.neighbors = []
        # DOWN
        if self.row < self.total_cols - 1 and not grid[self.row + 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col])
        # UP
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col])
        # RIGHT
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col + 1])
        # LEFT
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row][self.col - 1])
    
    # if less than = True, return false
    def __lt__(self, other):
        return False

def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw):
    path = [[current.row, current.col]]
    while current in came_from:
        current = came_from[current]
        path.append([current.row, current.col])
        current.make_path()
        draw()
    print(path)
    return np.array(path)

def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue() # gives minimum element of queue
    open_set.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    # we can't see what is in priority queue, so we keep track of what is in it here
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2] # get node
        open_set_hash.remove(current) # synchronize

        if current == end:
            path = reconstruct_path(came_from, end, draw)
            end.make_end()
            return path
        
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current  
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    return False

def make_grid(rows, columns, width, height):
    grid = []
    gap_r = height // rows # integer division
    gap_c = width // columns
    for i in range(columns):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap_c, gap_r, rows, columns)
            grid[i].append(spot)
    return grid

def draw_grid(win, rows, columns, width, height):
    gap_c = width // columns
    gap_r = height // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i*gap_r), (width, i*gap_r))
    for j in range(columns):
        pygame.draw.line(win, GREY, (j*gap_c, 0), (j*gap_c, height))

def draw(win, grid, rows, columns, width, height):
    win.fill(WHITE)  # resets screen on every frame

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, columns, width, height)
    pygame.display.update()

def get_clicked_pos(pos, rows, columns, width, height):
    gap_r = height // rows
    gap_c = width // columns 
    y,x = pos

    row = y // gap_r
    col = x // gap_c

    return row, col

def coord_to_pos(spot):

    x = spot.x + spot.width // 2
    y = spot.y + spot.height // 2

    return x, y

def main(width, height):
    win = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Path finding Algorithm")
    ROWS = 4
    COLUMNS = 7
    grid = make_grid(ROWS, COLUMNS, width, height)

    start = None
    end = None

    run = True
    started = False
    while run:
        draw(win, grid, ROWS, COLUMNS, width, height)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if started:
                continue

            if pygame.mouse.get_pressed()[0]: # left
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, COLUMNS, width, height)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()
                
                elif spot != end and spot != start:
                    spot.make_barrier()


            elif pygame.mouse.get_pressed()[2]: # right
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, COLUMNS, width, height)
                spot = grid[row][col]    
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None
                
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)
                    path = algorithm(lambda: draw(win, grid, ROWS, COLUMNS, width, height), grid, start, end)
                    
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, COLUMNS, width, height)
                     

    pygame.quit()
    return path

if __name__ == '__main__':
    main(WIDTH, HEIGHT)