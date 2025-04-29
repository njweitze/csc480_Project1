import sys

# list of valid robot actions
ACTIONS = ['N', 'S', 'E', 'W', 'V']

# mapping of direction actions to row/column movement
MOVES = {
    'N': (-1, 0),
    'S': (1, 0),
    'E': (0, 1),
    'W': (0, -1)
}

# depth-first search implementation
def dfs(initial_state):
    stack = [(initial_state, [])]  # (state, path_so_far)
    visited = set()
    nodes_generated = 1
    nodes_expanded = 0

    while stack:
        state, path = stack.pop()
        nodes_expanded += 1

        robot_pos, dirty = state

        # if no dirty cells remain, return the path
        if len(dirty) == 0:
            return path, nodes_generated, nodes_expanded

        # use sorted tuple of dirty cells to avoid hash issues
        state_id = (robot_pos, tuple(sorted(dirty)))
        if state_id in visited:
            continue
        visited.add(state_id)

        # generate successors from all possible actions
        for action in ACTIONS:
            new_robot_pos, new_dirty = move(robot_pos, dirty, action)
            if new_robot_pos is not None:
                new_state = (new_robot_pos, new_dirty)
                stack.append((new_state, path + [action]))
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded

# uniform-cost search implementation
def ucs(initial_state):
    frontier = [(0, initial_state, [])]  # (total_cost, state, path_so_far)
    visited = set()
    nodes_generated = 1
    nodes_expanded = 0

    while frontier:
        # manually find node with lowest cost
        lowest_index = 0
        for i in range(1, len(frontier)):
            if frontier[i][0] < frontier[lowest_index][0]:
                lowest_index = i
        cost, state, path = frontier.pop(lowest_index)
        nodes_expanded += 1

        robot_pos, dirty = state

        # if no dirty cells remain, return optimal path
        if len(dirty) == 0:
            return path, nodes_generated, nodes_expanded

        state_id = (robot_pos, tuple(sorted(dirty)))
        if state_id in visited:
            continue
        visited.add(state_id)

        # generate and enqueue valid successor states
        for action in ACTIONS:
            new_robot_pos, new_dirty = move(robot_pos, dirty, action)
            if new_robot_pos is not None:
                new_state = (new_robot_pos, new_dirty)
                frontier.append((cost + 1, new_state, path + [action]))
                nodes_generated += 1

    return None, nodes_generated, nodes_expanded

# computes a new state based on a robot action
def move(robot_pos, dirty, action):
    r, c = robot_pos
    new_r, new_c = r, c

    if action == 'V':
        # vacuum: remove current cell from dirty set if dirty
        if robot_pos in dirty:
            new_dirty = dirty.copy()
            new_dirty.remove(robot_pos)
            return (robot_pos, new_dirty)
        else:
            return None, None  # invalid vacuum (not dirty)
    else:
        # move: calculate new position and validate it's not blocked
        dr, dc = MOVES[action]
        new_r, new_c = r + dr, c + dc

        if 0 <= new_r < len(GRID) and 0 <= new_c < len(GRID[0]):
            if GRID[new_r][new_c] != '#':
                return ((new_r, new_c), dirty)

    return None, None  # move out of bounds or into wall

# extracts robot position and dirty cell locations from grid
def find_robot_and_dirty(grid):
    dirty = set()
    robot_pos = None
    for r, row in enumerate(grid):
        for c, cell in enumerate(row):
            if cell == '@':
                robot_pos = (r, c)
            elif cell == '*':
                dirty.add((r, c))
    return robot_pos, dirty

# entry point for program
def main():
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py <algorithm> <world_file>")
        sys.exit(1)

    algorithm = sys.argv[1]
    world_file = sys.argv[2]

    try:
        # load world file and build grid
        with open(world_file, 'r', encoding='utf-16') as f:
            columns = int(f.readline().strip())
            rows = int(f.readline().strip())

            global GRID
            GRID = []
            for _ in range(rows):
                line = f.readline().strip()
                GRID.append(list(line))

        # initialize search state
        robot_pos, dirty = find_robot_and_dirty(GRID)
        initial_state = (robot_pos, dirty)

        # run requested search algorithm
        if algorithm == "depth-first":
            path, nodes_generated, nodes_expanded = dfs(initial_state)
        elif algorithm == "uniform-cost":
            path, nodes_generated, nodes_expanded = ucs(initial_state)
        else:
            print("Unknown algorithm. Use 'depth-first' or 'uniform-cost'.")
            sys.exit(1)

        # output result
        if path is None:
            print("No path found.")
        else:
            for action in path:
                print(action)
            print(f"{nodes_generated} nodes generated")
            print(f"{nodes_expanded} nodes expanded")

    except FileNotFoundError:
        print(f"Error: File '{world_file}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

# start program
if __name__ == "__main__":
    main()
