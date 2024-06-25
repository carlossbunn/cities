import networkx as nx
import matplotlib.pyplot as plt
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

def held_karp_tsp(G):
    # Initialization
    n = len(G.nodes)
    nodes = list(G.nodes)
    index = {node: i for i, node in enumerate(nodes)}
    dp = [[float('inf')] * n for _ in range(1 << n)]
    dp[1][0] = 0
    parent = [[-1] * n for _ in range(1 << n)]

    # Dynamic Programming
    for mask in range(1 << n):
        for u in range(n):
            if mask & (1 << u):
                for v in range(n):
                    if not (mask & (1 << v)):
                        new_mask = mask | (1 << v)
                        weight = G[nodes[u]][nodes[v]]['weight']
                        if dp[new_mask][v] > dp[mask][u] + weight:
                            dp[new_mask][v] = dp[mask][u] + weight
                            parent[new_mask][v] = u


    mask = (1 << n) - 1
    min_cost = float('inf')
    last_node = -1
    for u in range(1, n):
        cost = dp[mask][u] + G[nodes[u]][nodes[0]]['weight']
        if cost < min_cost:
            min_cost = cost
            last_node = u

   
    path = []
    node = last_node
    while node != -1:
        path.append(nodes[node])
        mask ^= (1 << node)
        node = parent[mask][node]

    path.append(nodes[0])
    path.reverse()

    return path, min_cost

def plot_graph(G, pos, path, filename):
    plt.figure(figsize=(15, 10))
    edge_labels = {(u, v): f"{G[u][v]['weight']:.2f} km" for u, v in G.edges}

    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=3000, font_size=15, font_weight='bold')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red', font_size=15)

    if path:
        edge_list = [(path[i], path[i+1]) for i in range(len(path)-1)]
        nx.draw_networkx_edges(G, pos, edgelist=edge_list, edge_color='blue', width=2)

    plt.savefig(filename)

if __name__ == "__main__":
    G = create_graph(cities_coordinates)
    optimal_path, optimal_cost = held_karp_tsp(G)
    print(f"Optimal path: {' -> '.join(optimal_path)} with cost: {optimal_cost:.2f} km")

    # Plotting the optimal path
    pos = nx.spring_layout(G)
    plot_graph(G, pos, optimal_path, "Optimal_Path.png")

