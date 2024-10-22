import heapq
import random
import pygame
import time

class DynamicMaze:
    def __init__(self, width, height, wall_change_prob=0.1, teleport_prob=0.05):
        self.width = width
        self.height = height
        self.wall_change_prob = wall_change_prob
        self.teleport_prob = teleport_prob
        self.maze = [[0 for _ in range(width)] for _ in range(height)]
        self.teleport_points = []

    def initialize_maze(self):
        for y in range(self.height):
            for x in range(self.width):
                if random.random() < 0.3:  # 30% chance of being a wall
                    self.maze[y][x] = 1
                elif random.random() < 0.05:  # 5% chance of being a teleport point
                    self.maze[y][x] = 2
                    self.teleport_points.append((x, y))

    def get_neighbors(self, x, y):
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height and self.maze[ny][nx] != 1:
                neighbors.append((nx, ny))
        return neighbors

    def update_maze(self):
        for y in range(self.height):
            for x in range(self.width):
                if random.random() < self.wall_change_prob:
                    self.maze[y][x] = 1 - self.maze[y][x]  # Flip between wall and empty
                elif random.random() < self.teleport_prob:
                    self.maze[y][x] = 2 if self.maze[y][x] != 2 else 0
                    if self.maze[y][x] == 2:
                        self.teleport_points.append((x, y))
                    else:
                        self.teleport_points.remove((x, y))

def heuristic(a, b):
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def dynamic_astar(maze, start, goal):
    def get_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return get_path(came_from, current)

        for neighbor in maze.get_neighbors(*current):
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

        if maze.maze[current[1]][current[0]] == 2:
            teleport_dest = random.choice(maze.teleport_points)
            if teleport_dest not in g_score or g_score[current] + 1 < g_score[teleport_dest]:
                came_from[teleport_dest] = current
                g_score[teleport_dest] = g_score[current] + 1
                f_score[teleport_dest] = g_score[teleport_dest] + heuristic(teleport_dest, goal)
                heapq.heappush(open_set, (f_score[teleport_dest], teleport_dest))

        maze.update_maze()

    return None  # No path found

def draw_maze(screen, maze, path=None):
    cell_size = 20
    for y in range(maze.height):
        for x in range(maze.width):
            rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
            if maze.maze[y][x] == 1:
                pygame.draw.rect(screen, (0, 0, 0), rect)  # Wall
            elif maze.maze[y][x] == 2:
                pygame.draw.rect(screen, (0, 255, 255), rect)  # Teleport point
            else:
                pygame.draw.rect(screen, (255, 255, 255), rect)  # Empty space
            pygame.draw.rect(screen, (200, 200, 200), rect, 1)  # Grid lines

    if path:
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            start_pos = (start[0] * cell_size + cell_size // 2, start[1] * cell_size + cell_size // 2)
            end_pos = (end[0] * cell_size + cell_size // 2, end[1] * cell_size + cell_size // 2)
            pygame.draw.line(screen, (255, 0, 0), start_pos, end_pos, 3)

    pygame.display.flip()

def adaptive_multi_point_dynamic_astar(width, height, wall_change_prob, teleport_prob, starts, goals):
    pygame.init()
    screen = pygame.display.set_mode((width * 20, height * 20))
    pygame.display.set_caption("Dynamic Maze A* Pathfinding")

    maze = DynamicMaze(width, height, wall_change_prob, teleport_prob)
    maze.initialize_maze()
    
    starts = [(min(x, width-1), min(y, height-1)) for x, y in starts]
    goals = [(min(x, width-1), min(y, height-1)) for x, y in goals]
    
    best_path = None
    best_length = float('inf')

    for start in starts:
        for goal in goals:
            path = dynamic_astar(maze, start, goal)
            if path and len(path) < best_length:
                best_path = path
                best_length = len(path)
            
            draw_maze(screen, maze, best_path)
            time.sleep(0.5)  # Pause to show the result

    pygame.quit()
    return best_path, maze

def run_simulation(width, height, wall_change_prob, teleport_prob, starts, goals):
    starts = [(min(x, width-1), min(y, height-1)) for x, y in starts]
    goals = [(min(x, width-1), min(y, height-1)) for x, y in goals]
    
    path, maze = adaptive_multi_point_dynamic_astar(width, height, wall_change_prob, teleport_prob, starts, goals)
    if path:
        print(f"Path found for maze {width}x{height}, wall_change_prob={wall_change_prob}, teleport_prob={teleport_prob}")
        print(f"Path length: {len(path)}")
        print(f"Path: {path}")
    else:
        print(f"No path found for maze {width}x{height}, wall_change_prob={wall_change_prob}, teleport_prob={teleport_prob}")
    print("Teleport points:", maze.teleport_points)
    print("\n")

# Run simulations with different parameters
starts = [(0, 0), (0, 19), (19, 0)]
goals = [(19, 19), (10, 10), (15, 15)]

run_simulation(20, 20, 0.1, 0.05, starts, goals)
run_simulation(30, 30, 0.2, 0.1, starts, goals)
run_simulation(15, 15, 0.05, 0.02, starts, goals)

print("Changing parameters dynamically:")
width, height = 25, 25
wall_change_prob, teleport_prob = 0.15, 0.07
run_simulation(width, height, wall_change_prob, teleport_prob, starts, goals)

width, height = 35, 35
run_simulation(width, height, wall_change_prob, teleport_prob, starts, goals)

wall_change_prob = 0.25
run_simulation(width, height, wall_change_prob, teleport_prob, starts, goals)

teleport_prob = 0.12
run_simulation(width, height, wall_change_prob, teleport_prob, starts, goals)
