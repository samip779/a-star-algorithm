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

# This function calculates heuristic value using Manhatten's distance formula.


def heuristic(p1: Location, p2: Location) -> int:
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1-y2)


def find_neighbours(state: Location, terrain_map: Map, terrain_threshold: int):
    row, column = state
    neighbours = []

    # DOWN
    if row < len(terrain_map) - 1 and terrain_map[row + 1][column] < terrain_threshold:
        neighbours.append((row + 1, column))

    # UP
    if row > 0 and terrain_map[row-1][column] < terrain_threshold:
        neighbours.append((row - 1, column))

    # RIGHT
    if column < len(terrain_map[row]) - 1 and terrain_map[row][column + 1] < terrain_threshold:
        neighbours.append((row, column + 1))

    # LEFT
    if column > 0 and terrain_map[row][column - 1] < terrain_threshold:
        neighbours.append((row, column - 1))

    return neighbours


def find_probability(e):
    return 1 - e/100


def algorithm(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int, success_map: Map, success_threshold: float):

    count = 0
    frontier = PriorityQueue()
    frontier.put((0, count, start))
    log_enqueue_state(start, heuristic(start, goal),
                      find_probability(success_map[start]))
    came_from = {}
    for row in range(len(terrain_map)):
        for col in range(len(terrain_map[row])):
            came_from[(row, col)] = ()

    g_score = {}
    f_score = {}
    probability = {}
    dequeed = []
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
        log_visit_state(current, f_score[current], probability[current])

        if current == goal:
            result = reconstruct_path(came_from, goal)
            result.append(goal)
            result.pop(0)
            return [g_score[goal], probability[goal], result]
        isenqued = False
        neighbors = find_neighbours(current, terrain_map, terrain_threshold)
        if len(neighbors) == 0:
            return None
        for neighbour in neighbors:
            temp_g_score = g_score[current] + \
                terrain_map[current] + terrain_map[neighbour]
            temp_prob = probability[current] * \
                find_probability(success_map[neighbour])
            temp_f_score = temp_g_score + heuristic(neighbour, goal)

            if temp_f_score < f_score[neighbour] and temp_prob >= success_threshold:
                isenqued = True
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score

                f_score[neighbour] = temp_f_score
                probability[neighbour] = temp_prob
                if neighbour not in frontier_hash:
                    count += 1
                    frontier.put((f_score[neighbour], count, neighbour))
                    log_enqueue_state(
                        neighbour, f_score[neighbour], probability[neighbour])
                    frontier_hash.add(neighbour)
            elif temp_f_score >= f_score[neighbour] and temp_prob >= success_threshold:
                dequeed.append((f_score[neighbour], count, neighbour))
                log_ignore_state(neighbour, temp_f_score, temp_prob)

        if isenqued == False:

            erase_g_score(current, g_score, came_from,
                          terrain_map, terrain_threshold)

        if frontier.empty():
            for item in dequeed:
                frontier.put(item)
            dequeed.clear()

    return None


def erase_g_score(current, g_score, came_from, terrain_map, terrain_threshold):
    iterate = True
    while iterate:
        g_score[current] = float("inf")
        parent = came_from[current]
        came_from[current] = ()
        neighbours = find_neighbours(parent, terrain_map, terrain_threshold)

        for neighbour in neighbours:
            if came_from[neighbour] == parent:
                iterate = False
                break
            else:
                continue
        if iterate:
            current = parent


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
    try:
        return algorithm(start, goal, terrain_map, terrain_threshold, success_map, success_threshold)
    except:
        return None


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
