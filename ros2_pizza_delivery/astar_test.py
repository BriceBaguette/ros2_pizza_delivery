import numpy as np

class path_node:
    id: int
    position: tuple
    cost_to_node = dict

def calculate_cost_to_node(start: path_node, goal: path_node, map: list):
    init_pos = start.position
    end_pos = goal.position
    cost = get_path_cost(init_pos,end_pos,map)
    start.cost_to_node[goal.id] = cost
    goal.cost_to_node[start.id] = cost

def get_path_cost(a: tuple,b: tuple,map: list):
    best_cost= float('inf')
    directions = [(1,0),(-1,0),(0,1),(0,-1)]
    for i in range(len(directions)):
        new_pos = (a[0]+directions[i][0],a[1]+directions[i][1])
        if(map[new_pos[1]][new_pos[0]] == 0):
            cost = 1 + get_path_cost(new_pos,b,map)
            if(cost < best_cost):
                best_cost = cost
    return best_cost

def calculate_nodes(waypoints, map):
    return

map = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

waypoints = [(2, 2), (4, 2), (2, 4)]
start = (0, 0)
