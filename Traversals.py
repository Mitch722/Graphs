from Graph import Graph

import numpy as np


def main():

    no_nodes = 6
    graph_values = np.ones(no_nodes)

    node_names = ['S', 'A', 'B', 'C', 'D', 'E']

    A = np.zeros([no_nodes, no_nodes])
    # Fill out upper triangle of A to save typing
    A[0, 1] = 1.0
    A[0, 2] = 1.0
    A[3, 0] = 1.0

    A[1, 4] = 1.0

    A[2, 4] = 1.0
    
    A[4, 3] = 1.0

    A[3, 5] = 1.0

    # Create an undirected version 
    # A += A.transpose()

    graph = Graph(A, node_values=graph_values, node_names=node_names)

    visited_nodes = graph.dfs()
    print("=================")
    for node in visited_nodes:
        print(node)

    visited_nodes = graph.bfs()
    print("=================")
    for node in visited_nodes:
        print(node)

    visited_nodes = graph.topological_sort()
    print("=================")
    for node in visited_nodes:
        print(node)

    return 0




if __name__ == "__main__":
    main()
