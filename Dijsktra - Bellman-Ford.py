# Importar la librería matplotlib para graficar
import matplotlib.pyplot as plt
# Importar la librería random para generar pesos al azar
import random

# Definir una clase para representar un grafo dirigido con pesos
class Grafo:
  # Inicializar el grafo con un diccionario vacío
  def __init__(self):
    self.grafo = {}

  # Agregar un nodo al grafo con un diccionario vacío como valor
  def agregar_nodo(self, nodo):
    if nodo not in self.grafo:
      self.grafo[nodo] = {}

  # Agregar una arista al grafo con el peso como valor
  def agregar_arista(self, origen, destino, peso):
    self.agregar_nodo(origen) # Asegurarse de que el nodo origen exista
    self.agregar_nodo(destino) # Asegurarse de que el nodo destino exista
    self.grafo[origen][destino] = peso # Agregar la arista con el peso

  # Obtener los nodos del grafo
  def obtener_nodos(self):
    return list(self.grafo.keys())

  # Obtener las aristas del grafo
  def obtener_aristas(self):
    aristas = []
    for origen in self.grafo:
      for destino in self.grafo[origen]:
        aristas.append((origen, destino, self.grafo[origen][destino]))
    return aristas

  # Aplicar el algoritmo de Bellman-Ford para obtener la distancia más corta y el camino más corto desde un nodo origen a todos los demás nodos
  def bellman_ford(self, origen):
    # Inicializar la distancia y el camino para cada nodo
    distancia = {nodo: float("inf") for nodo in self.grafo} # Infinito para todos los nodos excepto el origen
    distancia[origen] = 0 # Cero para el origen
    camino = {nodo: [origen] for nodo in self.grafo} # El origen como primer elemento de la lista

    # Obtener las aristas del grafo
    aristas = self.obtener_aristas()

    # Relajar las aristas |V| - 1 veces, donde |V| es el número de nodos
    for i in range(len(self.grafo) - 1):
      # Para cada arista (origen, destino, peso)
      for u, v, w in aristas:
        # Si la distancia al destino es mayor que la distancia al origen más el peso de la arista
        if distancia[v] > distancia[u] + w:
          # Actualizar la distancia al destino
          distancia[v] = distancia[u] + w
          # Actualizar el camino al destino
          camino[v] = camino[u] + [v]

    # Verificar si hay ciclos negativos
    for u, v, w in aristas:
      if distancia[v] > distancia[u] + w:
        raise Exception("El grafo contiene un ciclo negativo")

    # Retornar la distancia y el camino para cada nodo
    return distancia, camino

  # Aplicar el algoritmo de Dijkstra para obtener la distancia más corta y el camino más corto desde un nodo origen a todos los demás nodos
  def dijkstra(self, origen):
    # Inicializar la distancia y el camino para cada nodo
    distancia = {nodo: float("inf") for nodo in self.grafo} # Infinito para todos los nodos excepto el origen
    distancia[origen] = 0 # Cero para el origen
    camino = {nodo: [origen] for nodo in self.grafo} # El origen como primer elemento de la lista

    # Inicializar un conjunto vacío para almacenar los nodos visitados
    visitados = set()

    # Mientras haya nodos no visitados en el grafo
    while len(visitados) < len(self.grafo):
      # Elegir el nodo no visitado con la menor distancia al origen
      min_nodo = None
      min_dist = float("inf")
      for nodo in self.grafo:
        if nodo not in visitados and distancia[nodo] < min_dist:
          min_nodo = nodo
          min_dist = distancia[nodo]

      # Si no se encontró ningún nodo, romper el bucle
      if min_nodo is None:
        break

      # Marcar el nodo elegido como visitado
      visitados.add(min_nodo)

      # Para cada vecino del nodo elegido
      for vecino in self.grafo[min_nodo]:
        # Si la distancia al vecino es mayor que la distancia al nodo elegido más el peso de la arista
        if distancia[vecino] > distancia[min_nodo] + self.grafo[min_nodo][vecino]:
          # Actualizar la distancia al vecino
          distancia[vecino] = distancia[min_nodo] + self.grafo[min_nodo][vecino]
          # Actualizar el camino al vecino
          camino[vecino] = camino[min_nodo] + [vecino]

    # Retornar la distancia y el camino para cada nodo
    return distancia, camino

# Crear un objeto de la clase Grafo
G = Grafo()

# Pedir al usuario que ingrese el número de vértices de la topología
n = int(input("Ingrese el número de vértices de la topología: "))

# Validar que el número de vértices sea positivo
while n <= 0:
  print("El número de vértices debe ser positivo. Por favor, ingrese un número válido.")
  n = int(input("Ingrese el número de vértices de la topología: "))

# Agregar los vértices al grafo con letras mayúsculas consecutivas
vertices = []
for i in range(n):
  letra = chr(ord("A") + i) # Obtener la letra mayúscula correspondiente al índice i
  vertices.append(letra) # Agregar la letra a la lista de vértices
  G.agregar_nodo(letra) # Agregar el vértice al grafo

# Pedir al usuario que ingrese el número de aristas de la topología
m = int(input("Ingrese el número de aristas de la topología: "))

# Validar que el número de aristas sea positivo y menor o igual al máximo número de aristas posibles
max_aristas = n * (n - 1) # El máximo número de aristas posibles es n * (n - 1) para un grafo dirigido
while m <= 0 or m > max_aristas:
  print(f"El número de aristas debe ser positivo y menor o igual a {max_aristas}. Por favor, ingrese un número válido.")
  m = int(input("Ingrese el número de aristas de la topología: "))

# Pedir al usuario que ingrese las aristas y sus pesos
for i in range(m):
  # Pedir al usuario que ingrese el vértice origen de la arista i
  origen = input(f"Ingrese el vértice origen de la arista {i + 1}: ")

  # Validar que el vértice origen sea válido
  while origen not in vertices:
    print("El vértice origen no es válido. Por favor, ingrese un vértice que pertenezca a la topología.")
    origen = input(f"Ingrese el vértice origen de la arista {i + 1}: ")

  # Pedir al usuario que ingrese el vértice destino de la arista i
  destino = input(f"Ingrese el vértice destino de la arista {i + 1}: ")

  # Validar que el vértice destino sea válido y diferente del vértice origen
  while destino not in vertices or destino == origen:
    print("El vértice destino no es válido o es igual al vértice origen. Por favor, ingrese un vértice que pertenezca a la topología y sea diferente del vértice origen.")
    destino = input(f"Ingrese el vértice destino de la arista {i + 1}: ")

  # Pedir al usuario que ingrese el peso de la arista i
  peso = int(input(f"Ingrese el peso de la arista {i + 1}: "))

  # Validar que el peso sea positivo
  while peso <= 0:
    print("El peso debe ser positivo. Por favor, ingrese un número válido.")
    peso = int(input(f"Ingrese el peso de la arista {i + 1}: "))

  # Agregar la arista y su peso al grafo
  G.agregar_arista(origen, destino, peso)

# Graficar el grafo con los pesos de las aristas usando la librería NetworkX solo para la gráfica
import networkx as nx
H = nx.DiGraph() # Crear un grafo dirigido vacío de NetworkX
H.add_weighted_edges_from(G.obtener_aristas()) # Agregar las aristas con los pesos desde el grafo G
pos = nx.spring_layout(H) # Definir una posición para los nodos
nx.draw(H, pos, with_labels=True) # Dibujar los nodos y las etiquetas
labels = nx.get_edge_attributes(H, "weight") # Obtener los pesos de las aristas
nx.draw_networkx_edge_labels(H, pos, edge_labels=labels) # Dibujar los pesos de las aristas
plt.show() # Mostrar la gráfica

# Pedir al usuario que ingrese el nodo origen
origen = input("Ingrese el nodo origen: ")

# Validar que el nodo origen sea válido
while origen not in vertices:
  print("El nodo origen no es válido. Por favor, ingrese un nodo que pertenezca a la topología.")
  origen = input("Ingrese el nodo origen: ")

# Pedir al usuario que ingrese el nodo destino
destino = input("Ingrese el nodo destino: ")

# Validar que el nodo destino sea válido y diferente del nodo origen
while destino not in vertices or destino == origen:
  print("El nodo destino no es válido o es igual al nodo origen. Por favor, ingrese un nodo que pertenezca a la topología y sea diferente del nodo origen.")
  destino = input("Ingrese el nodo destino: ")

# Aplicar el algoritmo de Bellman-Ford para obtener la distancia más corta y el camino más corto desde el nodo origen a todos los demás nodos
distancia_bf, camino_bf = G.bellman_ford(origen) # Aplicar el algoritmo de Bellman-Ford
print(f"El nodo origen es: {origen}") # Imprimir el nodo origen
print(f"La distancia más corta desde el nodo origen a cada nodo usando Bellman-Ford es: {distancia_bf}") # Imprimir la distancia más corta usando Bellman-Ford
print(f"El camino más corto desde el nodo origen a cada nodo usando Bellman-Ford es: {camino_bf}") # Imprimir el camino más corto usando Bellman-Ford

# Aplicar el algoritmo de Dijkstra para obtener la distancia más corta y el camino más corto desde el nodo origen a todos los demás nodos
distancia_dj, camino_dj = G.dijkstra(origen) # Aplicar el algoritmo de Dijkstra
print(f"La distancia más corta desde el nodo origen a cada nodo usando Dijkstra es: {distancia_dj}") # Imprimir la distancia más corta usando Dijkstra
print(f"El camino más corto desde el nodo origen a cada nodo usando Dijkstra es: {camino_dj}") # Imprimir el camino más corto usando Dijkstra

# Imprimir la mejor ruta entre el nodo origen y el nodo destino usando ambos algoritmos
print(f"La mejor ruta entre {origen} y {destino} usando Bellman-Ford es: {camino_bf[destino]}")
print(f"La mejor ruta entre {origen} y {destino} usando Dijkstra es: {camino_dj[destino]}")

# Graficar la distancia más corta entre el origen y el destino usando la librería NetworkX solo para la gráfica
I = nx.DiGraph() # Crear un grafo dirigido vacío de NetworkX
I.add_weighted_edges_from(G.obtener_aristas()) # Agregar las aristas con los pesos desde el grafo G
pos = nx.spring_layout(I) # Definir una posición para los nodos

# Obtener la lista de nodos en la mejor ruta usando Bellman-Ford
ruta_bf = camino_bf[destino]

# Obtener la lista de nodos en la mejor ruta usando Dijkstra
ruta_dj = camino_dj[destino]

# Dibujar los nodos y las etiquetas del grafo completo
nx.draw(I, pos, with_labels=True)

# Dibujar los nodos y las aristas de la mejor ruta usando Bellman-Ford con otro color y grosor
nx.draw_networkx_nodes(I, pos, nodelist=ruta_bf, node_color="red")
nx.draw_networkx_edges(I, pos, edgelist=[(ruta_bf[i], ruta_bf[i+1]) for i in range(len(ruta_bf) - 1)], edge_color="red", width=3)

# Dibujar los nodos y las aristas de la mejor ruta usando Dijkstra con otro color y grosor
nx.draw_networkx_nodes(I, pos, nodelist=ruta_dj, node_color="green")
nx.draw_networkx_edges(I, pos, edgelist=[(ruta_dj[i], ruta_dj[i+1]) for i in range(len(ruta_dj) - 1)], edge_color="green", width=3)

# Mostrar la gráfica
plt.show()
