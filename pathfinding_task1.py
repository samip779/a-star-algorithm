from calendar import c
from os import remove
import click
from typing import Optional
from events import log, log_visit_state, log_enqueue_state, log_ignore_state
from maps import Location, Map
from parsing import validate_location, validate_map
from queue import PriorityQueue


def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        current = came_from[current]
        path.append(current)
    reversed_path = path[::-1]
    reversed_path.pop(0)
    return reversed_path


# this function calculates heuristic value using Manhatten's distance formula
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


def algorithm(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int):
    count = 0
    frontier = PriorityQueue()
    frontier.put((0, count, start))
    log_enqueue_state(start, heuristic(start, goal))
    came_from = {}

    for row in range(len(terrain_map)):
        for col in range(len(terrain_map[row])):
            came_from[(row, col)] = ()

    g_score = {}
    f_score = {}
    dequeed = []

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
            return [g_score[goal], result]

        isenqued = False

        neighbors = find_neighbours(current, terrain_map, terrain_threshold)
        if len(neighbors) == 0:
            return None

        for neighbour in neighbors:
            temp_g_score = g_score[current] + \
                terrain_map[current] + terrain_map[neighbour]
            temp_f_score = temp_g_score + heuristic(neighbour, goal)

            if temp_f_score < f_score[neighbour]:
                isenqued = True
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + heuristic(neighbour, goal)
                if neighbour not in frontier_hash:
                    count += 1
                    frontier.put((f_score[neighbour], count, neighbour))
                    log_enqueue_state(neighbour, f_score[neighbour])
                    frontier_hash.add(neighbour)
            elif temp_f_score >= f_score[neighbour]:
                dequeed.append((f_score[neighbour], count, neighbour))
                log_ignore_state(neighbour, temp_f_score)

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


def find_shortest_path(start: Location, goal: Location,
                       terrain_map: Map, terrain_threshold: int) \
        -> tuple[Optional[int], Optional[list[Location]]]:
    """Finds the path with lowest total cost (Task 1)
       Returns (cost,list(locations)) when a path is found.
       Returns (None,None) if no path is found."""

    # This is the entry point for your code for Task 1.
    # Please create additional functions and classes etc as needed
    # to structure your implementation.
    # Avoid implementing the entire algorithm in one long chunk.
    try:
        return algorithm(start, goal, terrain_map, terrain_threshold)
    except:
        return None


@click.command(no_args_is_help=True)
@click.argument('start', required=True, callback=validate_location)
@click.argument('goal', required=True, callback=validate_location)
@click.argument("terrain_map", required=True, type=click.Path(exists=True), callback=validate_map)
@click.argument("terrain_threshold", required=True, type=click.IntRange(min=0, max=1000))
def main(start: Location, goal: Location, terrain_map: Map, terrain_threshold: int) -> None:
    """Example usage:

    \b
    python pathfinding_task1.py 3,2 0,3 resources/terrain01.txt 50
    """
    path = find_shortest_path(start, goal, terrain_map, terrain_threshold)
    if path:
        log(f"The path is {path[1]} with cost {path[0]}.")
    else:
        log('No path found')


if __name__ == '__main__':
    main()
