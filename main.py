from utils import *
from grid import Grid
from searching_algorithms import *

from utils import GRID_SIZE, SIDEBAR_WIDTH, TOTAL_WIDTH
import random

# Simple Button helper
class Button:
    def __init__(self, x, y, w, h, text, color=(200, 200, 200)):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.color = color

    def draw(self, win, font):
        pygame.draw.rect(win, self.color, self.rect)
        txt_s = font.render(self.text, True, (0, 0, 0))
        txt_rect = txt_s.get_rect(center=self.rect.center)
        win.blit(txt_s, txt_rect)

    def is_hover(self, pos):
        return self.rect.collidepoint(pos)


if __name__ == "__main__":
    pygame.init()  # initialize pygame (needed for fonts, audio, etc.)
    # setting up how big will be the display window
    WIN = pygame.display.set_mode((TOTAL_WIDTH, HEIGHT))

    # set a caption for the window
    pygame.display.set_caption("Path Visualizing Algorithm")

    ROWS = 50  # number of rows
    COLS = 50  # number of columns
    grid = Grid(WIN, ROWS, COLS, GRID_SIZE, HEIGHT)

    start = None
    end = None

    # UI: buttons
    FONT = pygame.font.SysFont(None, 20)
    BTN_W, BTN_H = SIDEBAR_WIDTH - 20, 30  # Fit sidebar width
    PADDING = 10

    # Create separate buttons for each algorithm, stacked vertically
    algo_buttons = []
    current_y = PADDING
    algorithms = ["BFS", "DFS", "A*_Manhattan","A*_Euclidian","DLS","UCS","Dijkstra","IDS","IDA"]  # extend as needed
    for alg in algorithms:
        btn = Button(GRID_SIZE + PADDING, current_y, BTN_W, BTN_H, alg)
        algo_buttons.append(btn)
        current_y += BTN_H + PADDING // 2  # Tight spacing

    # Maze button (to match Figure 1)
    maze_btn = Button(GRID_SIZE + PADDING, current_y, BTN_W, BTN_H, "MAZE")
    current_y += BTN_H + PADDING // 2

    # Clear button at bottom
    clear_btn = Button(GRID_SIZE + PADDING, current_y, BTN_W, BTN_H, "CLEAR")

    # NEW ADDITION: Define grid rect for partial updates (only update grid area in loop)
    grid_rect = pygame.Rect(0, 0, GRID_SIZE, HEIGHT)

    # flags for running the main loop
    run = True
    started = False

    # NEW ADDITION: Draw sidebar and buttons ONCE before the loop (static, no flicker)
    # Draw sidebar background
    pygame.draw.rect(WIN, COLORS['GREY'], (GRID_SIZE, 0, SIDEBAR_WIDTH, HEIGHT))
    # Draw buttons
    for btn in algo_buttons:
        btn.draw(WIN, FONT)
    maze_btn.draw(WIN, FONT)
    clear_btn.draw(WIN, FONT)
    pygame.display.update()  # Initial full update
    start = None
    end = None
    grid.reset()
    #NEW ADDITION: Redraw grid after key clear
    grid.draw()

    # OLD CODE: (drawing was inside loop; now moved out for no flicker)
    # # Draw sidebar background (grey like grid lines, to separate from grid)
    # pygame.draw.rect(WIN, COLORS['GREY'], (GRID_SIZE, 0, SIDEBAR_WIDTH, HEIGHT))
    #
    # grid.draw()  # draw the grid and its spots
    #
    # # Draw buttons on sidebar
    # for btn in algo_buttons:
    #     btn.draw(WIN, FONT)
    # maze_btn.draw(WIN, FONT)
    # clear_btn.draw(WIN, FONT)
    #
    # pygame.display.update()  # Update display AFTER everything is drawn

    while run:
        # NEW REPLACEMENT: No full redraw here; only draw grid if changed (e.g., in events), and update only grid_rect
        # grid.draw() is now called only on changes below, with internal update()

        for event in pygame.event.get():
            # verify what events happened
            if event.type == pygame.QUIT:
                run = False

            # handle mouse button events for buttons (use event-based clicks)
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                pos = event.pos

                # Check algo buttons
                for btn in algo_buttons:
                    if btn.is_hover(pos) and not started:
                        if not start or not end:
                            continue  # Missing endpoints
                        # Prepare grid
                        for row in grid.grid:
                            for spot in row:
                                spot.update_neighbors(grid.grid)
                        started = True
                        sel = btn.text
                        if sel == "BFS":
                            bfs(lambda: grid.draw(), grid, start, end)
                        elif sel == "DFS":
                            dfs(lambda: grid.draw(), grid, start, end)
                        elif sel == "A*_Manhattan":
                            astar(lambda: grid.draw(), grid, start, end, True)
                        elif sel == "A*_Euclidian":
                            astar(lambda: grid.draw(), grid, start, end, False)
                        elif sel == "DLS":
                            dls(lambda: grid.draw(), grid, start, end, limit=200)
                        elif sel == "UCS":
                            ucs(lambda: grid.draw(), grid, start, end)
                        elif sel == "Dijkstra":
                            dijkstra_search(lambda: grid.draw(), grid, start, end)
                        elif sel == "IDS":
                            ids(lambda: grid.draw(), grid, start, end, max_depth=50)
                        elif sel == "IDA":
                            ida(lambda: grid.draw(), grid, start, end)
                        started = False
                        # NEW ADDITION: After algo, redraw grid once (buttons unchanged)
                        grid.draw()

                # Maze button: Generate random barriers
                if maze_btn.is_hover(pos):
                    for row in grid.grid:
                        for spot in row:
                            if spot != start and spot != end and random.random() < 0.3:  # 30% chance of barrier
                                spot.make_barrier()
                            elif spot != start and spot != end:
                                spot.reset()  # Clear non-barriers
                    # NEW ADDITION: Redraw grid after maze gen (buttons unchanged)
                    grid.draw()

                # Clear button
                if clear_btn.is_hover(pos):
                    # clear the grid
                    start = None
                    end = None
                    grid.reset()
                    # NEW ADDITION: After clear, redraw grid (buttons unchanged)
                    grid.draw()

            if started:
                # do not allow any other interaction if the algorithm has started
                continue  # ignore other events if algorithm started

            if pygame.mouse.get_pressed()[0]:  # LEFT CLICK for grid edits
                pos = pygame.mouse.get_pos()
                if pos[0] >= GRID_SIZE:  # Ignore sidebar
                    continue

                row, col = grid.get_clicked_pos(pos)

                if row >= ROWS or row < 0 or col >= COLS or col < 0:
                    continue  # ignore clicks outside the grid

                spot = grid.grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()
                # NEW ADDITION: Update only grid after change (real-time drawing, no button flicker)
                grid.draw()  # Calls internal update()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                if pos[0] >= GRID_SIZE:  # Ignore sidebar
                    continue

                row, col = grid.get_clicked_pos(pos)
                spot = grid.grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None
                # NEW ADDITION: Update only grid after change
                grid.draw()  # Calls internal update()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    print("Clearing the grid...")
                    start = None
                    end = None
                    grid.reset()
                    # NEW ADDITION: Redraw grid after key clear
                    grid.draw()
    pygame.quit()