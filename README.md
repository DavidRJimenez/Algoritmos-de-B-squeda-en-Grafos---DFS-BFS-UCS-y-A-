# 🔍 Algoritmos de Búsqueda en Grafos - DFS, BFS, UCS y A*


Este proyecto implementa y visualiza los algoritmos clásicos de búsqueda en grafos: DFS (Búsqueda en Profundidad), BFS (Búsqueda en Amplitud), UCS (Búsqueda de Costo Uniforme) y A*. El usuario puede ingresar un grafo dirigido con pesos, especificar un nodo de inicio y destino, y comparar rutas y tiempos de ejecución entre los algoritmos.
🚀 Características

    Entrada personalizada del grafo (nodos y aristas con pesos).

    Implementación de 4 algoritmos de búsqueda:

        DFS

        BFS

        UCS

        A* con heurística trivial

    Visualización del grafo y caminos encontrados con colores por algoritmo.

    Comparación gráfica de los tiempos de ejecución.

    Interfaz interactiva por consola.

📸 Ejemplo de visualización

Cada camino encontrado por un algoritmo se resalta con un color específico:

    DFS - Morado

    BFS - Verde

    UCS - Rojo

    A* - Azul

También se muestra una gráfica de barras con los tiempos de ejecución para cada algoritmo.
🛠️ Requisitos

    Python 3.x

    matplotlib

    networkx

    (Opcional) pygraphviz para mejorar el layout del grafo

Puedes instalar las dependencias con:

pip install matplotlib networkx pygraphviz

    Si pygraphviz no está disponible, el programa usará un layout alternativo automáticamente.

▶️ Uso

Ejecuta el script y sigue las instrucciones por consola:

python grafo_busqueda.py

    Ingresa el número de nodos.

    Especifica los nombres de los nodos.

    Ingresa las aristas con el formato nodo1 nodo2 peso.

    Especifica el nodo de inicio y de destino.

    Selecciona qué caminos visualizar.

📈 Comparación de Algoritmos

El programa mide y grafica el tiempo de ejecución de cada algoritmo en segundos, permitiendo una comparación clara entre su rendimiento.
📚 Créditos

Desarrollado con fines educativos para demostrar la diferencia entre algoritmos de búsqueda en grafos, tanto en su comportamiento como en su rendimiento.