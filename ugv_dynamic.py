import random
import heapq

SIZE = 20
SENSOR_RANGE = 3
MAX_STEPS = 200


def create_grid(size, density, start, goal):
    grid = []
    for i in range(size):
        row = []
        for j in range(size):
            if (i, j) == start or (i, j) == goal:
                row.append(".")
            elif random.random() < density:
                row.append("X")
            else:
                row.append(".")
        grid.append(row)
    return grid


def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])


def get_neighbors(grid, current, blocked):
    x, y = current
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]):
            if grid[nx][ny] != "X" and (nx, ny) not in blocked:
                neighbors.append((nx, ny))
    return neighbors


def a_star(grid, start, goal, blocked=()):
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    nodes_explored = 0

    while priority_queue:
        f, current = heapq.heappop(priority_queue)
        nodes_explored += 1

        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            return path, nodes_explored

        for neighbor in get_neighbors(grid, current, blocked):
            new_g = g_cost[current] + 1
            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                f = new_g + heuristic(neighbor, goal)
                parent[neighbor] = current
                heapq.heappush(priority_queue, (f, neighbor))

    return None, nodes_explored


def sense(real_grid, known_grid, moving, position):
    """UGV sees only cells within SENSOR_RANGE of its position."""
    x, y = position
    for i in range(SIZE):
        for j in range(SIZE):
            if abs(i - x) + abs(j - y) <= SENSOR_RANGE and real_grid[i][j] == "X":
                known_grid[i][j] = "X"          # static obstacle discovered (remembered)
    # moving obstacles are not remembered, only those currently visible
    visible = set()
    for m in moving:
        if abs(m[0] - x) + abs(m[1] - y) <= SENSOR_RANGE:
            visible.add(m)
    return visible


def move_obstacles(real_grid, moving):
    """Each moving obstacle takes one random step."""
    for k in range(len(moving)):
        x, y = moving[k]
        dx, dy = random.choice([(-1, 0), (1, 0), (0, -1), (0, 1), (0, 0)])
        nx, ny = x + dx, y + dy
        if 0 <= nx < SIZE and 0 <= ny < SIZE and real_grid[nx][ny] != "X":
            moving[k] = (nx, ny)


def print_grid(real_grid, trail, moving, start, goal):
    trail_set = set(trail)
    for i in range(SIZE):
        row = ""
        for j in range(SIZE):
            if (i, j) == start:
                row += "S "
            elif (i, j) == goal:
                row += "G "
            elif (i, j) in moving:
                row += "M "
            elif (i, j) in trail_set:
                row += "* "
            else:
                row += real_grid[i][j] + " "
        print(row)


# --------------------------------------------------
# MAIN PROGRAM
# --------------------------------------------------

print("UGV A* PATH FINDING - DYNAMIC OBSTACLES")

print("\nEnter Start Position")
start = (int(input("Start row (0-19): ")), int(input("Start column (0-19): ")))

print("\nEnter Goal Position")
goal = (int(input("Goal row (0-19): ")), int(input("Goal column (0-19): ")))

print("\nSelect Obstacle Density")
print("1. Low    2. Medium    3. High")
choice = input("Enter your choice (1/2/3): ")

if choice == "1":
    density_name, density, n_moving = "Low", 0.10, 3
elif choice == "2":
    density_name, density, n_moving = "Medium", 0.20, 6
elif choice == "3":
    density_name, density, n_moving = "High", 0.30, 10
else:
    print("Invalid choice.")
    exit()

# Real world (UGV cannot see all of it) and what the UGV knows
real_grid = create_grid(SIZE, density, start, goal)
known_grid = [["."] * SIZE for _ in range(SIZE)]

# Moving obstacles placed on free cells away from the start
moving = []
while len(moving) < n_moving:
    cell = (random.randint(0, SIZE - 1), random.randint(0, SIZE - 1))
    if real_grid[cell[0]][cell[1]] == "." and heuristic(cell, start) > 3 and cell != goal:
        moving.append(cell)

# Reference: shortest path if everything static was known in advance
best_path, _ = a_star(real_grid, start, goal)

position = start
trail = [start]
plan = None
replans = waits = collisions = total_nodes = 0

for step in range(MAX_STEPS):
    if position == goal:
        break

    # 1. Sense
    visible = sense(real_grid, known_grid, moving, position)

    # block visible moving obstacles and the cells around them (safety margin)
    blocked = set()
    for (mx, my) in visible:
        for dx, dy in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)]:
            blocked.add((mx + dx, my + dy))
    blocked.discard(position)

    # 2. Replan if there is no plan or the plan is now blocked
    plan_blocked = plan is not None and any(
        c in blocked or known_grid[c[0]][c[1]] == "X" for c in plan[1:])
    if plan is None or plan_blocked:
        plan, nodes = a_star(known_grid, position, goal, blocked)
        total_nodes += nodes
        replans += 1

    # 3. Move one step (or wait if no safe path)
    if plan is None or len(plan) < 2:
        waits += 1
        plan = None
    else:
        plan = plan[1:]
        position = plan[0]
    trail.append(position)

    # 4. World changes
    move_obstacles(real_grid, moving)
    if position in moving:
        collisions += 1


print("\n")
print("=" * 50)
print("UGV BATTLEFIELD")
print("Obstacle Density:", density_name)
print("S=start  G=goal  X=static obstacle  M=moving obstacle  *=UGV trail")
print("=" * 50)
print()
print_grid(real_grid, trail, moving, start, goal)

print("\n")
print("=" * 50)
print("MEASURES OF EFFECTIVENESS")
print("=" * 50)

if position == goal:
    steps = len(trail) - 1 - waits
    print("Goal Reached         : Yes")
    print("Distance Travelled   :", steps)
    if best_path:
        print("Shortest (all known) :", len(best_path) - 1)
        print("Efficiency           : %.1f %%" % (100 * (len(best_path) - 1) / steps))
    print("Replans              :", replans)
    print("Waits                :", waits)
    print("Collisions           :", collisions)
    print("Nodes Explored       :", total_nodes)
else:
    print("Goal Reached         : No (stopped after", MAX_STEPS, "steps)")
    print("Replans              :", replans)
    print("Collisions           :", collisions)