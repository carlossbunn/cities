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
    'Rome': (41.9028, 12.4964)
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
