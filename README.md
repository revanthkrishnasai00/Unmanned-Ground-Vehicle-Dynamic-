Here is your documentation for the dynamic and unknown obstacle UGV navigation project formatted into a clean, professional, copy-pasteable Markdown template optimized for your GitHub `README.md` file.

```markdown
# UGV Navigation with Dynamic and Unknown Obstacles using A* with Replanning

## 1. Problem Statement
In static path planning problems, all obstacles are fixed and fully known in advance. In the real world, this assumption fails:
* Some obstacles are **unknown** until the UGV's sensors detect them.
* Some obstacles are **dynamic** (moving), meaning a previously clear path can become blocked unexpectedly.

**Goal:** Ensure the UGV reaches the goal safely and efficiently by as short a route as possible, even though the map is only partly known and changes continuously over time.

---

## 2. Key Idea: Sense, Plan, Move, Repeat
A single offline plan cannot stay valid in a changing environment. Instead, the UGV operates in a continuous loop, maintaining a **belief map** of what it currently knows and replanning whenever that belief changes.

At every time step:
1. **Sense:** Detect obstacles within sensor range and update the belief map.
2. **Check:** Determine if any cell on the current plan is now blocked.
3. **Replan:** If yes (or if no active plan exists), run A* from the current position to the goal.
4. **Move:** Take one step along the plan, or wait if no safe path exists.
5. **World Changes:** Every moving obstacle takes one random step.

The loop ends when the UGV reaches the goal or hits the 200-step limit.

---

## 3. Real World vs. Belief Map
The program manages two distinct grid spaces:
* **`real_grid`:** The true state of static obstacles in the environment (partially hidden from the UGV).
* **`known_grid`:** What the UGV currently believes the map looks like. It starts completely empty and fills in progressively as obstacles are sensed.

### Handling Uncertainty
* **Static Obstacles:** When hidden static obstacles come within sensor range, they are added to `known_grid` permanently.
* **Moving Obstacles:** Because their positions change every step, they are **not remembered long-term**. Only those currently within sensor range are treated as blocked cells, together with a surrounding safety margin of cells to prevent close encounters.

---

## 4. State-Space Formulation

| Element | Definition |
| :--- | :--- |
| **State** | `(row, column)`: the cell the UGV occupies |
| **Initial state** | The UGV's current cell (changes dynamically at each replan) |
| **Goal test** | `current == goal` |
| **Actions** | Move up, down, left, or right (4 directions) |
| **Transition model** | Illegal if the move leaves the grid, enters a cell the UGV believes is a static obstacle, or enters a blocked zone around a visible moving obstacle. |
| **Step cost** | 1 per move |
| **Path cost $g(n)$** | Number of moves from the current position |

*Note: The UGV plans strictly on its belief map, not on the true world.*

---

## 5. Algorithm: A* with Replanning
Each plan is generated using standard A* with:

$$f(n) = g(n) + h(n)$$

* **$g(n)$**: Actual cost from the current position to cell $n$.
* **$h(n)$**: Estimated cost to the goal using **Manhattan distance** ($\vert{}x_1 - x_2\vert{} + \vert{}y_1 - y_2\vert{}$).

Replanning is triggered *only* when the remaining active plan becomes blocked, avoiding needless recomputation.

### Pseudocode Outline
```python
known_grid = empty_grid()
position = start
plan = None

while position != goal and steps < 200:
  # 1. Sense environment
  update_known_grid_with_visible_static()
  visible_moving_obstacles = get_visible_moving_with_margin()

  # 2. Check if current plan is invalidated
  if plan is None or any(cell in plan for cell in visible_moving_obstacles):
    plan = a_star(known_grid, position, goal, visible_moving_obstacles)

  # 3. Move or wait
  if plan has a valid next step:
    position = plan.next_step()
  else:
    wait_one_step()

  # 4. World state updates
  move_all_dynamic_obstacles()
  check_collisions()

```

---

## 6. Code Structure (`ugv_dynamic_simple.py`) and Settings

| Function / Component | Purpose |
| --- | --- |
| `create_grid()` | Builds the true environment grid with random static obstacles |
| `heuristic()` | Calculates Manhattan distance to the goal |
| `get_neighbors()` | Returns legal next cells while avoiding blocked zones |
| `a_star()` | Runs search; returns the computed path and nodes explored |
| `sense()` | Updates `known_grid` and identifies visible moving obstacles |
| `move_obstacles()` | Advances each dynamic obstacle by one random step |
| **Main Program** | Executes the sense-replan-move loop and prints performance results |

### Simulation Settings

* **Grid Dimensions:** $20 \times 20$ cells
* **Sensor Range:** 3 cells (Manhattan distance)
* **Step Limit:** 200 steps

### Difficulty Levels

| Level | Static Obstacle Density | Moving Obstacles Count |
| --- | --- | --- |
| **Low** | 10% | 3 |
| **Medium** | 20% | 6 |
| **High** | 30% | 10 |

*(The performance reference baseline is the shortest possible path computed on the fully known real grid before execution).*

---

## 7. Measures of Effectiveness (MOEs)

| MOE | Meaning |
| --- | --- |
| **Goal reached** | Boolean indicator of whether the UGV arrived within the step limit |
| **Distance travelled** | Actual number of moves made (waiting steps excluded) |
| **Shortest (all known)** | Optimal path length if all static obstacles were known in advance |
| **Efficiency (%)** | $\text{Shortest} \div \text{distance travelled} \times 100$. 100% means zero extra travel. |
| **Replans** | Total number of times A* had to be re-run |
| **Waits** | Steps spent waiting because no safe path existed |
| **Collisions** | Number of times a moving obstacle stepped into the UGV's cell |
| **Nodes explored** | Cumulative cells examined by A* across all replanning phases |

---

## 8. Sample Results

*(Single trial runs from start `(0, 0)` to goal `(19, 19)`)*

| Metric | Low Density | Medium Density |
| --- | --- | --- |
| **Goal reached** | Yes | Yes |
| **Distance travelled** | 40 | 44 |
| **Shortest (all known)** | 38 | 38 |
| **Efficiency (%)** | 95.0 | 86.4 |
| **Replans** | 6 | 5 |
| **Waits** | 0 | 0 |
| **Collisions** | 0 | 0 |
| **Nodes explored** | 988 | 1186 |

### Observations

1. **Robust Navigation:** The UGV successfully reached the goal in both runs without collisions, proving that the sense-and-replan loop effectively handles unknown and moving obstacles.
2. **Path Overhead:** Actual travel distance exceeds the theoretical best-case path because the UGV only discovers static obstacles at close range, necessitating local detours.
3. **Search Effort:** Frequent replanning significantly increases cumulative nodes explored compared to static planning models.
4. **Efficiency Drop:** Efficiency decreases as obstacle density rises due to an increasing number of surprise detours.
5. **Dynamic Optimality:** In a changing world, "optimal" shifts from a rigid global solution to the best route possible given local real-time awareness.

---

## 9. How to Run

1. Ensure **Python 3** is installed (with *"Add python.exe to PATH"* checked).
2. Run the script directly from your terminal:
```bash
python ugv_dynamic_simple.py

```


3. Enter your parameters when prompted:
* Start row and column (0 to 19)
* Goal row and column (0 to 19)
* Obstacle density choice (1 for Low, 2 for Medium, 3 for High)


4. The terminal will output the runtime map and computed MOEs.
* **Map Key:** `S` = Start, `G` = Goal, `X` = Static Obstacle, `M` = Moving Obstacle (final position), `*` = Traveled Route.



---

## 10. Alternative Algorithms

| Algorithm | Core Idea | Best Suited For |
| --- | --- | --- |
| **A* with Replanning (Used Here)** | Rerun full A* whenever the active plan is blocked | Small maps; simple implementation |
| **D* Lite** | Searches backward from the goal and reuses past work, updating only affected cells | Large maps with unknown/changing terrain |
| **Space-Time A*** | Adds time as a state dimension $(x, y, t)$ to avoid predicted moving obstacles | Predictable obstacle trajectories |
| **Local Planners (e.g., DWA)** | React to nearby obstacles continuously in real-time | Fast-moving, reactive obstacles |

---

## 11. Assumptions and Limitations

* Moving obstacles follow a random walk (one cell per turn or staying still) and do not actively react to the UGV's presence.
* The UGV moves at parity speed with dynamic obstacles (one cell per step).
* Sensors provide a perfect 3-cell Manhattan radius view with zero noise or line-of-sight occlusions.
* All static obstacles are completely unknown at the start (no prior global map provided).
* Movement is restricted to 4 cardinal directions with uniform unit step costs.
* Collisions are logged for tracking but carry no physical execution penalty. A production setup would incorporate motion prediction or expanded safety margins.
* Results vary across runs due to randomized obstacle placement (add `random.seed(x)` near the top for repeatable testing).

```

```
