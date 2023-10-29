import heapq

"""
En este ejemplo, el algoritmo de Dijkstra se utiliza para encontrar las distancias más cortas
desde un nodo de inicio (en este caso, 'A') a todos los demás nodos en un grafo ponderado. 
El grafo se representa como un diccionario, donde las claves son los nodos y los valores son 
diccionarios que contienen los nodos vecinos y los pesos de las aristas. 
El algoritmo calcula las distancias más cortas y las muestra en la salida.
"""

def dijkstra(graph, start):
    # Inicializar las distancias a todos los nodos como infinito y la distancia al nodo de inicio como 0.
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # Crear una cola de prioridad (heap) para mantener los nodos a explorar.
    priority_queue = [(0, start)]

    while priority_queue:
        # Obtener el nodo con la distancia más corta.
        current_distance, current_node = heapq.heappop(priority_queue)

        # Si encontramos una distancia más larga, omitirla.
        if current_distance > distances[current_node]:
            continue

        # Explorar los nodos vecinos y actualizar sus distancias si encontramos un camino más corto.
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances

# Ejemplo de uso
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}
start_node = 'A'

shortest_distances = dijkstra(graph, start_node)
print("Distancias más cortas desde el nodo de inicio (", start_node, "):")
for node, distance in shortest_distances.items():
    print(node, ":", distance)
