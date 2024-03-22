"""
Al Muqshith Shifan #100862739
Hameem Quader  #100786057
Dammy Olude #100888626
Arshia Mortazavinezhad #100860353
"""


# Library's for reading CSV files
import csv
from collections import defaultdict

# ANSI escape codes for colors
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# Define a function to read the CSV file and return a list of edges
def read_csv(filename):
    with open(filename, newline='', encoding='utf-8') as csvfile:
        csv_reader = csv.reader(csvfile)
        edges = [(row[0], row[1], int(row[2])) for row in csv_reader]
    return edges

# Define a function to create an adjacency list from the list of edges
def create_graph(edges):
    graph = defaultdict(list)
    for from_node, to_node, weight in edges:
        graph[from_node].append((to_node, weight))
    return graph

# Define a function to print the graph
def print_graph(graph):
    for node in graph:
        print(f'{node} -> {graph[node]}')


def dijkstra(graph, start):
    """
    distances = {node: float('inf') for node in graph}: This initializes the distances dictionary, setting the distance to each node from the start node as infinity (float('inf')) because initially, we don't know the shortest distances.

    distances[start] = 0: The distance from the start node to itself is set to zero.

    paths = {node: [] for node in graph}: This creates a paths dictionary to store the actual shortest path (as a list of nodes) to each node.

    paths[start] = [start]: The path from the start node to itself is just a list containing the start node.

    visited = set(): This initializes an empty set to keep track of the visited nodes.

    """

    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    paths = {node: [] for node in graph}
    paths[start] = [start]

    visited = set()

    while True:
        # Find the unvisited node with the smallest distance
        closest_node = None
        closest_distance = float('inf')
        for node in distances:
            if node not in visited and distances[node] < closest_distance:
                closest_node = node
                closest_distance = distances[node]

        if closest_node is None:
            # There are no remaining nodes, or all remaining nodes are inaccessible from the start
            break

        # Update the distances and paths for neighbors of the closest node
        for neighbor, weight in graph[closest_node]:
            if neighbor not in visited:
                distance = closest_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    paths[neighbor] = paths[closest_node] + [neighbor]

        # Mark the closest node as visited
        visited.add(closest_node)

    return distances, paths



def recommend_efficient_route(charging_station_distances):
    # Find the charging station with the smallest distance
    closest_station = min(charging_station_distances, key=charging_station_distances.get)
    distance = charging_station_distances[closest_station]
    return closest_station, distance

def Shortest_path_Algorithm(graph, charging_stations):
    for start_node in graph:
        if start_node not in charging_stations:
            print(f"\n{Colors.UNDERLINE}Paths from Node {start_node}:{Colors.ENDC}")
            distances, paths = dijkstra(graph, start_node)

            charging_station_paths = {station: paths[station] for station in charging_stations if station != start_node}
            charging_station_distances = {station: distances[station] for station in charging_stations if station != start_node}

            print(f"{Colors.BLUE}Shortest Paths to Charging Stations from Node {start_node}:{Colors.ENDC}")
            for station, path in charging_station_paths.items():
                print(f"{Colors.GREEN}To Station {station}: {' -> '.join(path)} (Distance: {charging_station_distances[station]}){Colors.ENDC}")

            closest_station, closest_distance = recommend_efficient_route(charging_station_distances)
            recommended_path = ' -> '.join(charging_station_paths[closest_station])
            print(f"{Colors.BOLD}{Colors.BLUE}Recommended Route:{Colors.ENDC}")
            print(f"{Colors.BOLD}{Colors.GREEN}The most efficient route from Node {start_node} is to Station {closest_station}, taking the path {recommended_path}, with a total distance of {closest_distance}.{Colors.ENDC}")


def main():
    edges = read_csv('EV.csv')  # Make sure to replace 'EV.csv' with the correct path to your CSV file
    graph = create_graph(edges)

    print(f"{Colors.HEADER}Electric Vehicle Charging Paths:{Colors.ENDC}")
    print_graph(graph)

    charging_stations = ['K', 'Q', 'T', 'V']
    Shortest_path_Algorithm(graph, charging_stations)

#Run's Application
main()