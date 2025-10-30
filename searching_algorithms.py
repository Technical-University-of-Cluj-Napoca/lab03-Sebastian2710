from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot
from math import inf


def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Breadth-First Search (BFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None: return False
    queue = deque()
    queue.append(start)
    visited = {start}
    came_from = {}
    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        else:
            current=queue.popleft()
            if current == end:
                while current in came_from:
                    current = came_from[current]
                    current.make_path()
                    draw()
                end.make_end()
                start.make_start()
                return True
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    neighbor.make_open()
                    queue.append(neighbor)
            draw()
            if current != start:
                current.make_closed()
    return False

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Depdth-First Search (DFS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None: return False
    stack = []
    stack.append(start)
    visited = {start}
    came_from = {}
    while stack:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
        else:
            current=stack.pop()
            if current == end:
                while current in came_from:
                    current = came_from[current]
                    current.make_path()
                    draw()
                end.make_end()
                start.make_start()
                return True
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    neighbor.make_open()
                    stack.append(neighbor)
            draw()
            if current != start:
                current.make_closed()
    return False

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Euclidian distance between p1 and p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def astar(draw: callable, grid: Grid, start: Spot, end: Spot,manhattan:bool) -> bool:
    """
    A* Pathfinding Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))
    came_from = {}
    g_score = {spot: float("inf") for row in grid.grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid.grid for spot in row}
    if manhattan is True:
        f_score[start] = h_manhattan_distance((start.row, start.col), (end.row, end.col))
    else:
        f_score[start] = h_euclidian_distance((start.row, start.col), (end.row, end.col))
    open_set = {start}

    while not open_heap.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current = open_heap.get()[2]
        open_set.remove(current)

        if current == end:
            while current in came_from:
                current = came_from[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            tentative_g = g_score[current] + 1  # cost(current, neighbor) = 1
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                if manhattan is True:
                    f_score[neighbor] = tentative_g + h_manhattan_distance((neighbor.row, neighbor.col), (end.row, end.col))
                else:
                    f_score[neighbor] = tentative_g + h_euclidian_distance((neighbor.row, neighbor.col), (end.row, end.col))
                if neighbor not in open_set:
                    count += 1
                    open_heap.put((f_score[neighbor], count, neighbor))
                    open_set.add(neighbor)
                    neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()
    return False

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)
def dls(draw: callable, grid: Grid, start: Spot, end: Spot, limit: int) -> bool:
    """
    Depth-Limited Search (DLS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
        limit (int): The depth limit for the search.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False
    stack = [(start, 0)]
    visited = {start}
    came_from = {}
    while stack:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False
        current, depth = stack.pop()
        if current == end:
            # Reconstruct path without coloring start
            path_current = end
            while path_current in came_from:
                path_current = came_from[path_current]
                if path_current != start:
                    path_current.make_path()
                    draw()
            end.make_end()
            start.make_start()
            draw()  # Ensure final colors are drawn
            return True
        if depth < limit:
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    neighbor.make_open()
                    stack.append((neighbor, depth + 1))
        draw()
        if current != start:
            current.make_closed()
    return False

# ▢ Uninformed Cost Search (UCS)
def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Uninformed Cost Search (UCS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    if start is None or end is None:
        return False
    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))
    came_from = {}
    g_score = {start: 0}

    while not open_heap.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current_dist, _, current = open_heap.get()

        if current_dist > g_score.get(current, inf):
            continue

        if current == end:
            # Reconstruct path without coloring start
            path_current = end
            while path_current in came_from:
                path_current = came_from[path_current]
                if path_current != start:
                    path_current.make_path()
                    draw()
            end.make_end()
            start.make_start()
            draw()  # Ensure final colors are drawn
            return True

        for neighbor in current.neighbors:
            if neighbor.is_barrier():
                continue
            tentative_g = g_score[current] + 1  # Assume uniform cost; modify for varying costs
            old_g = g_score.get(neighbor, inf)
            if tentative_g < old_g:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                if old_g == inf:
                    neighbor.make_open()
                count += 1
                open_heap.put((tentative_g, count, neighbor))

        draw()
        if current != start:
            current.make_closed()
    return False

# ▢ Greedy Search
def dijkstra_search(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Dijkstra's Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    

    count = 0
    open_heap = PriorityQueue()
    open_heap.put((0, count, start))
    came_from = {}
    g_score = {start: 0}

    while not open_heap.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False

        current_dist, _, current = open_heap.get()

        if current_dist > g_score.get(current, inf):
            continue

        if current == end:
            # Reconstruct path without coloring start
            path_current = end
            while path_current in came_from:
                path_current = came_from[path_current]
                if path_current != start:
                    path_current.make_path()
                    draw()
            end.make_end()
            start.make_start()
            draw()  # Ensure final colors are drawn
            return True

        for neighbor in current.neighbors:
            if neighbor.is_barrier():
                continue
            tentative_g = g_score[current] + 1
            old_g = g_score.get(neighbor, inf)
            if tentative_g < old_g:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                if old_g == inf:
                    neighbor.make_open()
                count += 1
                open_heap.put((tentative_g, count, neighbor))

        draw()
        if current != start:
            current.make_closed()
    return False

# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)
def ids(draw: callable, grid: Grid, start: Spot, end: Spot, max_depth: int) -> bool:
    """
    Iterative Deepening Search (IDS) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
        max_depth (int): The maximum depth limit for the search.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    for depth in range(10,max_depth,10):
        # Reset grid visuals for each iteration (clear open/closed except start/end/barriers)
        for row in grid.grid:
            for spot in row:
                if not spot.is_start() and not spot.is_end() and not spot.is_barrier():
                    spot.reset()
        start.make_start()  # Reapply to ensure color persists
        end.make_end()      # Reapply to ensure color persists                    
        draw()  # Redraw clean grid
        if dls(draw, grid, start, end, depth):
            draw()  # Ensure final draw after path
            return True
    return False

# ▢ Iterative Deepening A* (IDA)
def ida(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    """
    Iterative Deepening A* (IDA*) Algorithm.
    Args:
        draw (callable): A function to call to update the Pygame window.
        grid (Grid): The Grid object containing the spots.
        start (Spot): The starting spot.
        end (Spot): The ending spot.
    Returns:
        bool: True if a path is found, False otherwise.
    """
    def ida_helper(current: Spot, g_cost: float, threshold: float, visited: set) -> float | bool:
        for event in pygame.event.get():  # Check events in recursion (may miss some)
            if event.type == pygame.QUIT:
                pygame.quit()
                return inf  # Treat as no path
        f_cost = g_cost + h_manhattan_distance((current.row, current.col), (end.row, end.col))
        if f_cost > threshold:
            return f_cost
        if current == end:
            return True
        visited.add(current)
        min_threshold = inf
        for neighbor in current.neighbors:
            if not neighbor.is_barrier() and neighbor not in visited:
                neighbor.make_open()
                draw()
                temp = ida_helper(neighbor, g_cost + 1, threshold, visited)  # Uniform cost=1
                if temp is True:
                    came_from[neighbor] = current
                    return True
                if isinstance(temp, float) and temp < min_threshold:
                    min_threshold = temp
        visited.remove(current)  # Backtrack
        if current != start:
            current.make_closed()
            draw()
        return min_threshold

    threshold = h_manhattan_distance((start.row, start.col), (end.row, end.col))
    came_from = {}
    while True:
        visited = set()
        temp = ida_helper(start, 0, threshold, visited)
        if temp is True:
            # Reconstruct path without coloring start
            path_current = end
            while path_current in came_from:
                path_current = came_from[path_current]
                if path_current != start:
                    path_current.make_path()
                    draw()
            end.make_end()
            start.make_start()
            draw()  # Ensure final colors
            return True
        if temp == inf:
            return False
        threshold = temp