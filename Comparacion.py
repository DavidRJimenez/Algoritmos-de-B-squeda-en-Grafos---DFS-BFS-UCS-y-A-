import heapq
import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
import time

def input_graph():
    graph = {}
    num_nodes = int(input("Ingrese el número de nodos: "))
    
    for _ in range(num_nodes):
        node = input("Ingrese el nombre del nodo: ").strip().upper()
        graph[node] = {}

    num_edges = int(input("Ingrese el número de aristas: "))

    for _ in range(num_edges):
        u, v, w = input("Ingrese una arista (nodo1 nodo2 peso): ").strip().upper().split()
        w = int(w)
        if u in graph and v in graph:
            graph[u][v] = w
        else:
            print(f"Error: Uno de los nodos {u} o {v} no existe.")

    return graph

def dfs(graph, start, goal, path=None, visited=None):
    if path is None:
        path = []
    if visited is None:
        visited = set()

    path.append(start)
    visited.add(start)

    if start == goal:
        return path

    for neighbor in graph[start]:
        if neighbor not in visited:
            result = dfs(graph, neighbor, goal, path[:], visited)
            if result:
                return result
    return None

def bfs(graph, start, goal):
    queue = deque([(start, [start])])

    while queue:
        node, path = queue.popleft()

        if node == goal:
            return path

        for neighbor in graph[node]:
            if neighbor not in path:
                queue.append((neighbor, path + [neighbor]))
    return None

def ucs(graph, start, goal):
    pq = [(0, start, [])]
    visited = set()

    while pq:
        cost, node, path = heapq.heappop(pq)

        if node in visited:
            continue
        visited.add(node)

        path = path + [node]
        if node == goal:
            return path, cost

        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path))

    return None, float("inf")

def astar(graph, start, goal):
    pq = [(0, start, [])]
    visited = set()
    heuristic = lambda x: 0  # Heurística trivial

    while pq:
        cost, node, path = heapq.heappop(pq)

        if node in visited:
            continue
        visited.add(node)

        path = path + [node]
        if node == goal:
            return path, cost

        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight + heuristic(neighbor), neighbor, path))
    
    return None, float("inf")

def draw_graph(graph, paths, selected_paths):
    G = nx.DiGraph()
    for node in graph:
        for neighbor, weight in graph[node].items():
            G.add_edge(node, neighbor, weight=weight)

    try:
        pos = nx.nx_agraph.graphviz_layout(G, prog="dot")  # Usa Graphviz para formato de árbol
    except:
        pos = nx.multipartite_layout(G)  # Alternativa si Graphviz no está instalado

    plt.figure(figsize=(8, 6))
    nx.draw(G, pos, with_labels=True, node_color="lightgray", edge_color="gray", node_size=1000, font_size=12)
    legend_handles = []

    colors = {"DFS": "purple", "BFS": "green", "UCS": "red", "A*": "blue"}
    for algo, path in paths.items():
        if selected_paths[algo] and path:
            edges = [(path[i], path[i+1]) for i in range(len(path)-1)]
            nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color=colors[algo], width=3.5)
            legend_handles.append(plt.Line2D([0], [0], color=colors[algo], lw=3.5, label=algo))

    plt.legend(handles=legend_handles)
    plt.show()

def plot_time_comparison(times):
    algorithms = list(times.keys())
    exec_times = list(times.values())

    plt.figure(figsize=(8, 5))
    colors = ["purple", "green", "red", "blue"]

    # Barras con colores distintos para cada algoritmo
    plt.bar(algorithms, exec_times, color=colors, alpha=0.75)
    
    # Etiquetas
    plt.xlabel("Algoritmos")
    plt.ylabel("Tiempo de Ejecución (s)")
    plt.title("Comparación de tiempos de búsqueda")
    
    # Mostrar valores en las barras
    for i, time in enumerate(exec_times):
        plt.text(i, time + 0.000002, f"{time:.6f}s", ha="center", fontsize=10, fontweight="bold")

    plt.grid(axis="y", linestyle="--", alpha=0.7)
    plt.show()

graph = input_graph()
start_node = input("Ingrese el nodo de inicio: ").strip().upper()
goal_node = input("Ingrese el nodo de destino: ").strip().upper()

if start_node not in graph or goal_node not in graph:
    print("Error: Nodo inválido.")
else:
    # Medición de tiempos de ejecución
    start_time = time.time()
    dfs_path = dfs(graph, start_node, goal_node)
    dfs_time = time.time() - start_time

    start_time = time.time()
    bfs_path = bfs(graph, start_node, goal_node)
    bfs_time = time.time() - start_time

    start_time = time.time()
    ucs_path, _ = ucs(graph, start_node, goal_node)
    ucs_time = time.time() - start_time

    start_time = time.time()
    astar_path, _ = astar(graph, start_node, goal_node)
    astar_time = time.time() - start_time

    # Imprimir los resultados con tiempos en formato claro
    print(f"DFS Path: {dfs_path} (Tiempo: {dfs_time:.6f} s)")
    print(f"BFS Path: {bfs_path} (Tiempo: {bfs_time:.6f} s)")
    print(f"UCS Path: {ucs_path} (Tiempo: {ucs_time:.6f} s)")
    print(f"A* Path: {astar_path} (Tiempo: {astar_time:.6f} s)")

    # Selección de caminos para visualizar
    print("\nSeleccione el camino a visualizar:")
    print("1. DFS - Morado")
    print("2. BFS - Verde")
    print("3. UCS - Rojo")
    print("4. A* - Azul")
    print("5. Todos")
    
    choice = input("Ingrese el número de su elección: ").strip()
    selected_paths = {"DFS": False, "BFS": False, "UCS": False, "A*": False}
    
    if choice == "1":
        selected_paths["DFS"] = True
    elif choice == "2":
        selected_paths["BFS"] = True
    elif choice == "3":
        selected_paths["UCS"] = True
    elif choice == "4":
        selected_paths["A*"] = True
    elif choice == "5":
        selected_paths = {key: True for key in selected_paths}
    else:
        print("Opción no válida, mostrando todos por defecto.")
        selected_paths = {key: True for key in selected_paths}

    paths = {"DFS": dfs_path, "BFS": bfs_path, "UCS": ucs_path, "A*": astar_path}
    
    # Mostrar el grafo primero
    draw_graph(graph, paths, selected_paths)

    # Después de cerrar el grafo, mostrar la comparación de tiempos
    execution_times = {
        "DFS": dfs_time,
        "BFS": bfs_time,
        "UCS": ucs_time,
        "A*": astar_time
    }
    plot_time_comparison(execution_times)
