import itertools
import networkx as nx
import matplotlib.pyplot as plt
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from geopy.distance import geodesic


cities_coordinates = {
    'Paris': (48.8566, 2.3522),
    'Berlin': (52.5200, 13.4050),
    'Madrid': (40.4168, -3.7038),
    'Rome': (41.9028, 12.4964),
    'London': (51.5074, -0.1278),
    'Amsterdam': (52.3676, 4.9041),
    'Brussels': (50.8503, 4.3517),
    'Vienna': (48.2082, 16.3738),
    'Prague': (50.0755, 14.4378),
    'Warsaw': (52.2297, 21.0122),
    'Budapest': (47.4979, 19.0402),
    'Copenhagen': (55.6761, 12.5683),
    'Stockholm': (59.3293, 18.0686),
    'Oslo': (59.9139, 10.7522),
    'Helsinki': (60.1695, 24.9354),
    'Athens': (37.9838, 23.7275),
    'Lisbon': (38.7223, -9.1393),
    'Dublin': (53.3498, -6.2603),
    'Zurich': (47.3769, 8.5417),
    'Moscow': (55.7558, 37.6173),
    'Istanbul': (41.0082, 28.9784),
    'Reykjavik': (64.1355, -21.8954),
    'Tallinn': (59.4370, 24.7535),
    'Riga': (56.9496, 24.1052)
}

def create_graph(cities_coords):
    G = nx.DiGraph()
    cities = list(cities_coords.keys())

    G.add_nodes_from(cities)

    for city1, coord1 in cities_coords.items():
        for city2, coord2 in cities_coords.items():
            if city1 != city2:
                distance = geodesic(coord1, coord2).kilometers
                G.add_edge(city1, city2, weight=distance)

    return G

def tsp_brute_force(G, start, paths_results, lock, thread_id):
    nodes = list(G.nodes)
    nodes.remove(start)
    min_path = None
    min_cost = float('inf')
    paths_tried = []

    for perm in itertools.permutations(nodes):
        current_cost = 0
        current_path = [start]
        current_node = start

        for next_node in perm:
            current_cost += G[current_node][next_node]['weight']
            current_node = next_node
            current_path.append(current_node)

        current_cost += G[current_node][start]['weight']
        current_path.append(start)

        if current_cost < min_cost:
            min_cost = current_cost
            min_path = current_path

        paths_tried.append((list(current_path), current_cost))

    with lock:
        paths_results.append((min_path, min_cost, thread_id, paths_tried))

def plot_graph(G, pos, path, fig, ax, filename):
    ax.clear()
    edge_labels = {(u, v): f"{G[u][v]['weight']:.2f} km" for u, v in G.edges}

    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=3000, font_size=15, font_weight='bold', ax=ax)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red', font_size=15, ax=ax)

    if path:
        edge_list = [(path[i], path[i+1]) for i in range(len(path)-1)]
        nx.draw_networkx_edges(G, pos, edgelist=edge_list, edge_color='blue', width=2, ax=ax)

    fig.savefig(filename)

if __name__ == "__main__":
    G = create_graph(cities_coordinates)
    start_city = 'Paris'
    paths_results = []
    lock = threading.Lock()

    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [executor.submit(tsp_brute_force, G, start_city, paths_results, lock, thread_id) for thread_id in range(4)]
        for future in futures:
            future.result()

    optimal_path, optimal_cost, _, _ = min(paths_results, key=lambda x: x[1])
    print(f"Optimal path: {' -> '.join(optimal_path)} with cost: {optimal_cost:.2f} km")

    # Plotting all paths tried by each thread
    pos = nx.spring_layout(G)
    for min_path, min_cost, thread_id, paths_tried in paths_results:
        fig, ax = plt.subplots(figsize=(15, 10))
        for step, (path, cost) in enumerate(paths_tried):
            plot_graph(G, pos, path, fig, ax, f"Thread_{thread_id}_Step_{step}.png")
