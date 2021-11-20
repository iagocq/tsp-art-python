"""Modified code from https://developers.google.com/optimization/routing/tsp#or-tools """
# Copyright Matthew Mack (c) 2020 under CC-BY 4.0: https://creativecommons.org/licenses/by/4.0/

from __future__ import print_function
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from PIL import Image, ImageDraw
import os
from itertools import permutations
import sys
from time import time

# Change these file names to the relevant files.
ORIGINAL_IMAGE = "images/logo-ufla.png"
IMAGE_TSP = "images/logo-ufla-1024-stipple.tsp"

def create_data_model():
    """Stores the data for the problem."""
    # Extracts coordinates from IMAGE_TSP and puts them into an array
    list_of_nodes = []
    with open(IMAGE_TSP) as f:
        for _ in range(6):
            next(f)
        for line in f:
            i,x,y = line.split()
            list_of_nodes.append((int(float(x)),int(float(y))))
    data = {}
    # Locations in block units
    data['locations'] = list_of_nodes # yapf: disable
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(
                    math.hypot((from_node[0] - to_node[0]),
                               (from_node[1] - to_node[1]))))
    return distances

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Objective: {}m\n'.format(route_distance)

def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
      index = routing.Start(route_nbr)
      route = [manager.IndexToNode(index)]
      while not routing.IsEnd(index):
        index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
      routes.append(route)
    return routes[0]

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    print("Step 1/5: Initialising variables")
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    print("Step 2/5: Computing distance matrix")
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    print("Step 3/5: Setting an initial solution")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    print("Step 4/5: Solving")
    start = time()
    solution = routing.SolveWithParameters(search_parameters)
    end = time()

    # Print solution on console.
    if solution:
        #print_solution(manager, routing, solution)
        print("Step 5/5: Drawing the solution")
        routes = get_routes(solution, routing, manager)
        draw_routes(data['locations'],routes)
        print(distancia(distance_matrix, routes), end-start)
    else:
        print("A solution couldn't be found :(")


def nearest_neighbor(inicial=0):
    data = create_data_model()

    m = compute_euclidean_distance_matrix(data['locations'])

    INF = float('inf')
    visitados = set()
    caminho = []

    start = time()

    u = inicial
    while True:
        caminho.append(u)
        visitados.add(u)

        menor = (-1, INF)
        for v in m:
            if v in visitados:
                continue

            w = m[u][v]
            if w < menor[1]:
                menor = (v, w)
        
        if menor[0] == -1:
            break

        u = menor[0]
    
    caminho.append(caminho[0])
    end = time()

    dt = distancia(m, caminho)
    print(dt, end-start)
    draw_routes(data['locations'], caminho, f'-nn-{dt}')

def perfect_n_path(inicial=0, n=5):
    data = create_data_model()

    m = compute_euclidean_distance_matrix(data['locations'])

    INF = float('inf')
    visitados = set()
    caminho = []

    start = time()

    u = inicial
    while True:
        proximos = set()
        visitados.add(u)
        caminho.append(u)

        fim = False
        atual = u
        for _ in range(n):
            menor = (-1, INF)
            for v in m:
                if v in visitados or v in proximos:
                    continue

                w = m[atual][v]
                if w < menor[1]:
                    menor = (v, w)

            atual = menor[0]
            if atual == -1:
                fim = True
                break

            proximos.add(atual)

        if fim:
            break

        if len(proximos) == 0:
            continue

        def menor_caminho(u, proximos, m):
            menor = ((), INF)
            for caminho in permutations(proximos):
                caminho = (u,) + caminho
                d = distancia(m, caminho)
                if d < menor[1]:
                    menor = (caminho, d)
            return menor[0]

        prox_caminho = menor_caminho(u, proximos, m)
        caminho.extend(prox_caminho)
        visitados.update(prox_caminho)
        u = prox_caminho[-1]

    caminho.append(caminho[0])
    end = time()

    d = distancia(m, caminho)
    print(n, d, end-start)
    draw_routes(data['locations'], caminho, f'-pnp4-{n}-{d}')

def distancia(m, caminho):
    d = 0
    u = caminho[0]
    for v in caminho[1:]:
        d += m[u][v]
        u = v
    return d

def draw_routes(nodes, path, suffix='-tsp', w=1):
    """Takes a set of nodes and a path, and outputs an image of the drawn TSP path"""
    tsp_path = []
    for location in path:
        tsp_path.append(nodes[int(location)])

    original_image = Image.open(ORIGINAL_IMAGE)
    width, height = original_image.size

    tsp_image = Image.new("RGBA",(width,height),color='white')
    tsp_image_draw = ImageDraw.Draw(tsp_image)
    #tsp_image_draw.point(tsp_path,fill='black')
    tsp_image_draw.line(tsp_path,fill='black',width=w)
    tsp_image = tsp_image.transpose(Image.FLIP_TOP_BOTTOM)
    FINAL_IMAGE = IMAGE_TSP.replace("-stipple.tsp", f'{suffix}-w{w}.png')
    tsp_image.save(FINAL_IMAGE)
    print("TSP solution has been drawn and can be viewed at", FINAL_IMAGE)

if __name__ == '__main__':
    main()
