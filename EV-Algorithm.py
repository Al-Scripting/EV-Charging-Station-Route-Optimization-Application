""""
Al Muqshith Shifan #100862739
Hameem Quader  #100786057
Dammy Olude #100888626
Arshia Mortazavinezhad #100860353
"""

# Import libraries for reading CSV files and working with data structures
import csv
from collections import defaultdict

# Define ANSI escape codes as constants for colorful console output
class Colors:
    HEADER = '\033[95m'  # Purple
    BLUE = '\033[94m'    # Blue
    GREEN = '\033[92m'   # Green
    WARNING = '\033[93m' # Yellow
    FAIL = '\033[91m'    # Red
    ENDC = '\033[0m'     # Reset to default
    BOLD = '\033[1m'     # Bold text
    UNDERLINE = '\033[4m'# Underline text

# Reads a CSV file and returns a list of edges (connections) between nodes
def read_csv(filename):
    with open(filename, newline='', encoding='utf-8') as csvfile:
        csv_reader = csv.reader(csvfile)
        # Extract edges with weights as tuples from the CSV rows
        edges = [(row[0], row[1], int(row[2])) for row in csv_reader]
    return edges

# Creates an adjacency list from a list of edges to represent the graph
def create_graph(edges):
    graph = defaultdict(list)
    # Fill the adjacency list with edges (node connections with weights)
    for from_node, to_node, weight in edges:
        graph[from_node].append((to_node, weight))
    return graph

# Prints the graph's adjacency list for visualization
def print_graph(graph):
    for node in graph:
        print(f'{node} -> {graph[node]}')

# Implements Dijkstra's algorithm to find the shortest paths from a start node
def dijkstra(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    paths = {node: [] for node in graph}
    paths[start] = [start]
    visited = set()

    while True:
        closest_node = None
        closest_distance = float('inf')
        # Find the unvisited node with the smallest known distance
        for node in distances:
            if node not in visited and distances[node] < closest_distance:
                closest_node = node
                closest_distance = distances[node]

        if closest_node is None:
            break  # No more nodes to visit or remaining nodes are inaccessible

        # Update distances and paths for neighbors of the closest node
        for neighbor, weight in graph[closest_node]:
            if neighbor not in visited:
                distance = closest_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    paths[neighbor] = paths[closest_node] + [neighbor]

        # Mark the closest node as visited
        visited.add(closest_node)

    return distances, paths

# Recommends the most efficient route to a charging station
def recommend_efficient_route(charging_station_distances):
    # Find the charging station with the shortest distance
    closest_station = min(charging_station_distances, key=charging_station_distances.get)
    distance = charging_station_distances[closest_station]
    return closest_station, distance

# Applies the shortest path algorithm and prints results for each start node
def Shortest_path_Algorithm(graph, charging_stations):
    for start_node in graph:
        if start_node not in charging_stations:
            print(f"\n{Colors.UNDERLINE}Paths from Node {start_node}:{Colors.ENDC}")
            distances, paths = dijkstra(graph, start_node)
            # Filter out paths and distances to charging stations only
            charging_station_paths = {station: paths[station] for station in charging_stations if station != start_node}
            charging_station_distances = {station: distances[station] for station in charging_stations if station != start_node}
            # Print shortest paths to charging stations
            print(f"{Colors.BLUE}Shortest Paths to Charging Stations from Node {start_node}:{Colors.ENDC}")
            for station, path in charging_station_paths.items():
                print(f"{Colors.GREEN}To Station {station}: {' -> '.join(path)} (Distance: {charging_station_distances[station]}){Colors.ENDC}")
            # Print the recommended route to the closest charging station
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