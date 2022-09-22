import click
from typing import Optional
from events import log, log_enqueue_state, log_ignore_state, log_visit_state
from maps import Location, Map
from parsing import validate_location, validate_map
from queue import PriorityQueue


def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        current = came_from[current]
        path.append(current)
    reversed_path = path[::-1]
    return reversed_path

# this function calculates heuristic value using Manhatten's distance formula
def heuristic(p1: Location, p2: Location) -> int:
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1-y2)


def find_neighbours(state: Location, terrain_map: Map, terrain_threshold: int, f_score):
    row, column = state
    neighbours = []

    # DOWN
    if row < len(terrain_map) - 1 and terrain_map[row + 1][column] < terrain_threshold:
        neighbours.append((row + 1, column))
    elif row < len(terrain_map) - 1 and terrain_map[row + 1][column] >= terrain_threshold:
        log_ignore_state((row + 1, column), f_score[row+1, column])
        pass

    # UP
    if row > 0 and terrain_map[row-1][column] < terrain_threshold:
        neighbours.append((row - 1, column))
    elif row > 0 and terrain_map[row - 1][column] >= terrain_threshold:
        log_ignore_state((row - 1, column), f_score[row-1, column])
        pass

    # LEFT
    if column < len(terrain_map[row]) - 1 and terrain_map[row][column + 1] < terrain_threshold:
        neighbours.append((row, column + 1))
    elif column < len(terrain_map[row]) - 1 and terrain_map[row][column + 1] >= terrain_threshold:
        log_ignore_state((row, column + 1), f_score[row, column + 1])
        pass

    # RIGHT
    if column > 0 and terrain_map[row][column - 1] < terrain_threshold:
        neighbours.append((row, column - 1))
    elif column > 0 and terrain_map[row][column - 1] >= terrain_threshold:
        log_ignore_state((row, column - 1), f_score[row, column - 1])
        pass

    return neighbours


def find_probability(e):
    return 1 - e/100


def algorithm(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int, success_map: Map, success_threshold: float):
 
    count = 0
    frontier = PriorityQueue()
    frontier.put((0, count, start))
    log_enqueue_state(start, heuristic(start, goal))
    came_from = {}

    g_score = {}
    f_score = {}
    probability = {}
    probability[start] = find_probability(success_map[start])

    for row in range(len(terrain_map)):
        for col in range(len(terrain_map[row])):
            g_score[(row, col)] = float("inf")

    g_score[start] = 0

    for row in range(len(terrain_map)):
        for col in range(len(terrain_map[row])):
            f_score[(row, col)] = float("inf")

    f_score[start] = heuristic(start, goal)

    frontier_hash = {start}

    while not frontier.empty():
        current = frontier.get()[2]
        frontier_hash.remove(current)
        log_visit_state(current, f_score[current])

        if current == goal:
            result = reconstruct_path(came_from, goal)
            result.append(goal)
            return [g_score[goal], probability[goal], result]

        for neighbour in find_neighbours(current, terrain_map, terrain_threshold, f_score):
            temp_g_score = g_score[current] + \
                terrain_map[current] + terrain_map[neighbour]
            temp_prob = probability[current] * \
                find_probability(success_map[neighbour])

            if temp_g_score < g_score[neighbour] and temp_prob >= success_threshold:

                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + heuristic(neighbour, goal)
                probability[neighbour] = temp_prob
                if neighbour not in frontier_hash:
                    count += 1
                    frontier.put((f_score[neighbour], count, neighbour))
                    log_enqueue_state(neighbour, f_score[neighbour])
                    frontier_hash.add(neighbour)

    return None


def find_shortest_safe_path(start: Location, goal: Location,
                            terrain_map: Map, terrain_threshold: int,
                            success_map: Map, success_threshold: float) \
        -> tuple[Optional[int], Optional[float], Optional[list[Location]]]:
    """Finds the path with lowest total cost that also satisfies
       the minimum success probability threshold (Task 2).
       Returns (cost,prob_success,list(locations)) when a path is found.
       Returns (None,None,None) if no path is found."""

    # This is the entry point for your code for Task 2.
    # Please create additional functions and classes etc as needed
    # to structure your implementation.
    # Avoid implementing the entire algorithm in one long chunk.

    return algorithm(start, goal, terrain_map, terrain_threshold, success_map, success_threshold)


@click.command(no_args_is_help=True)
@click.argument('start', required=True, callback=validate_location)
@click.argument('goal', required=True, callback=validate_location)
@click.argument("terrain_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0, max=1000))
@click.argument("success_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("success_threshold", required=True, type=click.FloatRange(min=0.0, max=1.0))
def main(start: Location, goal: Location,
         terrain_map: Map, success_map: Map,
         terrain_threshold: int, success_threshold: float) -> None:
    """Example usage:

        \b
        python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 1.0
    """
    path = find_shortest_safe_path(
        start, goal, terrain_map, terrain_threshold, success_map, success_threshold)
    if path:
        log(f"The path is {path[2]} with cost {path[0]} and success probability {path[1]}")
    else:
        log('No path found')


if __name__ == '__main__':
    main()
