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
    
    return reversed_path



def heuristic(p1: Location, p2: Location) -> int:
    # This function calculates heuristic value using Manhatten's distance formula.

    # It is consistent(Monotonic).
    # It fulfills the condition : h(n) <= h(m) + cost(n-m)
    # where m is a direct successor of n and h(g) = 0 for any goal state g
    
    # It always calculated the shortest distance between two points in a grid.

    # It is the sum of horizontal and vertical distance.
    # It is restricted to traverse diagonally in this map so Manhatten's
    # distance is prefered over other methods like Euclidean distance.
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

   

    g_score = {}
    f_score = {}
    

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

        

        neighbors = find_neighbours(current, terrain_map, terrain_threshold)
        

        for neighbour in neighbors:
            temp_g_score = g_score[current] + \
                terrain_map[current] + terrain_map[neighbour]
            temp_f_score = temp_g_score + heuristic(neighbour, goal)

            if temp_f_score < f_score[neighbour]:
                
                came_from[neighbour] = current
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + heuristic(neighbour, goal)
                if neighbour not in frontier_hash:
                    count += 1
                    frontier.put((f_score[neighbour], count, neighbour))
                    log_enqueue_state(neighbour, f_score[neighbour])
                    frontier_hash.add(neighbour)
            else:
                log_ignore_state(neighbour,temp_f_score)
            

           

   

    return None




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
