import math
import sys
import heapq

def euclidean_distance(intersection1_coord, intersection2_coord):
    #Function to calculate euclidean distance between two points
    x_diff = intersection2_coord[0] - intersection1_coord[0]
    y_diff = intersection2_coord[1] - intersection1_coord[1]
    
    return math.sqrt(x_diff * x_diff + y_diff * y_diff)
 
def shortest_path(M,start,goal):
    #Heap to store paths on frontier   
    frontier = []
    #Set to store unexplored nodes
    explored = set()
   
    step_cost = {}
    heuristic_cost = {}
    path_cost = {}
    
    #Compute step costs between connected neighbours for all intersections
    for intersection in M.intersections:
        for neighbour in M.roads[intersection]:
            step_cost[(neighbour, intersection)] = euclidean_distance(M.intersections[neighbour], M.intersections[intersection])  
        
    #Compute heuristic costs between intersection and goal
    for intersection in M.intersections:
        if (intersection == goal):
            heuristic_cost[intersection] = 0
        else:
            heuristic_cost[intersection] = euclidean_distance(M.intersections[intersection], M.intersections[goal])
    
    #Set path cost of start to 0 and infinity to all other nodes
    for intersection in M.intersections:
        if (intersection == start):
            path_cost[intersection] = 0
        else:
            path_cost[intersection] = sys.maxsize
    #Heap to store potential candidate solutions 
    path_list = []
    #Dictionary to store cheapest total cost at a particular node
    best_total_cost = {}
    
    #Push start 
    heapq.heappush(frontier, (path_cost[start] + heuristic_cost[start], [start]))
    
    while frontier:
        
        (output_cost, current_best_path) = heapq.heappop(frontier)
        current_node = current_best_path[-1]
        best_total_cost[current_node] = output_cost
        
        if current_node == goal:
            heapq.heappush(path_list, (best_total_cost[current_node], current_best_path))
        
        for neighbour in M.roads[current_node]:
            if neighbour not in explored: 
                path_cost[neighbour] = step_cost[(neighbour, current_node)] + path_cost[current_node]
                heapq.heappush(frontier, (path_cost[neighbour] + heuristic_cost[neighbour], current_best_path + [neighbour]))
            else:
                if (step_cost[(neighbour, current_node)] + path_cost[current_node] < path_cost[neighbour]):
                    path_cost[neighbour] = step_cost[(neighbour, current_node)] + path_cost[current_node]
                    heapq.heappush(frontier, (path_cost[neighbour] + heuristic_cost[neighbour], current_best_path + [neighbour]))
        
        explored.add(current_node)
    
    print("shortest path called")
    (best_path_cost, best_path) = heapq.heappop(path_list)
    return best_path