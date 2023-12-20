import numpy as np


class Node:
    """
    Node class containing the node's name, value and an array of connecting nodes
    """
    def __init__(self, name, value=1) -> None:
        self.name = name
        self.value = value
        self.connections = []

    def add_connection(self, node, directed=False) -> None:
        """
        Create a connection between self node and another

        Args:
            node: Node type class 
            directed: Bool, is the connection directed or undirected (default undirected)
        """
        self.connections.append(node)
        if not directed:
            node.connections.append(self)
        return
    
    def __str__(self) -> str:
        """
        Creates a string sufh as for the print statement
        """
        node_info = "Name: " + str(self.name) + " Value: " + str(self.value) + " Connections: " + str(len(self.connections))
        return node_info


class Graph:
    """
    Graph class initialised by an adjacency matrix
    """
    def __init__(self, adjacency_mat: np.array, node_values=None, node_names=None) -> None:
        self.adjacency_mat = adjacency_mat

        if node_values is not None:
            assert (len(node_values) == adjacency_mat.shape[0]), ("The number of node_values must equal the number " +
                                                                  "of rows in the adjacency matrix")
            self.node_values = node_values
        else:
            self.node_values = np.ones(adjacency_mat.shape[0])

        if node_names is not None:
            assert (len(node_values) == adjacency_mat.shape[0]), ("The number of node_names must equal the number " +
                                                                  "of rows in the adjacency matrix")
            self.node_names = node_names
        else:
            self.node_names = list(range(0, len(node_values), 1))   
        
        # Check if the graph is direct onr undirected
        self.undirected = (adjacency_mat == adjacency_mat.transpose()).all()

        # Create the Graph nodes and connect
        self.node_dict = self.create_graph()

    def create_graph(self) -> dict:
        """
        Creates the Nodes and connections for the graph from the adjacency matrix
        
        Args:
            self

        Returns:
            Dictionary containing the nodes as items and the node names as keys
        """
        node_dict = dict()
        for k, (name, value) in enumerate(zip(self.node_names, self.node_values)):
            node_dict[self.node_names[k]] = Node(name=name, value=value)
        for i in range(self.adjacency_mat.shape[0]):
            for j in range(self.adjacency_mat.shape[1]):
                if self.adjacency_mat[i, j] == 1:
                    # Keep nodes directed as True as we iterated over the entire
                    # adjacency matrix
                    node_dict[self.node_names[i]].add_connection(node_dict[self.node_names[j]], True)
        return node_dict

    def __str__(self) -> str:
        """
        Creates string for the print function
        """
        if self.undirected:
            directed = "undirected"
        else:
            directed = "directed"
        info = f"\nGraph is {directed} and has {len(self.node_dict.keys())} nodes: \n\n"
        for _, node in self.node_dict.items():
            info += f"Info: {str(node)} \n"
        return info
    
    def dfs(self, start_node=None):
        """
        Depth First Search through the graph

        Args:
            start_node: Node type class (default = None)

        Returns:
            visited_nodes: List type, returns list of Nodes in dfs order
        """
        # Create containers for tracking nodes and visited nodes
        stack = []
        visited_nodes = []
        # Start node
        if not start_node:
            start_node = self.node_dict[self.node_names[0]]
        # Initialise stack with first node
        stack.append(start_node)
        # Implement dfs
        def trav():
            while stack:
                node = stack[-1]
                stack.remove(node)
                if node not in visited_nodes:
                    visited_nodes.append(node)
                for connection in node.connections:
                    if connection not in visited_nodes:
                        stack.append(connection)
                        trav()
        trav()
        return visited_nodes
    
    def bfs(self, start_node=None):
        """
        Breadth First Search through the graph

        Args:
            start_node: Node type class (default = None)

        Returns:
            visited_nodes: List type, returns list of Nodes in bfs order
        """
        queue = []
        visited = []

        if not start_node:
            start_node = self.node_dict[self.node_names[0]]

        queue.append(start_node)

        def trav():
            while queue:
                node = queue[0]
                queue.remove(node)

                if node not in visited:
                    visited.append(node)

                for connection in node.connections:
                    if connection not in visited:
                        visited.append(connection)
                        if connection not in queue:
                            queue.append(connection)
                
                trav()
        trav()
        return visited

    def topological_sort(self):
        """
        Finds the topological order of graph including for cycles

        Args:
            start_node: Node type class (default = None)

        Returns:
            visited_nodes: List type, returns list of Nodes in topological order
        """
        # Keep track of nodes
        node_list = list(self.node_dict.values())

        stack = []
        sorted_order = []

        start_node = node_list[0]
        stack.append(start_node)
        
        def dfs(node):
            dfs_result = [node]
            cycle_dector = []
            def trav(node_):
                if node_.connections:
                    for connection in node_.connections:
                        if connection not in sorted_order:
                            if connection in cycle_dector:
                                return
                            cycle_dector.append(connection)
                            dfs_result.append(connection)
                            trav(dfs_result[-1])
                            break
                else:
                    return
            trav(node)
            return dfs_result

        def trav():
            while stack:
                node = stack.pop(-1)

                dfs_result = dfs(node)

                visited_node = dfs_result.pop(-1)
                stack.extend(dfs_result)
                sorted_order.append(visited_node)

                if visited_node in node_list:
                    node_list.remove(visited_node)
                if not stack and node_list:
                    new_node = node_list.pop(0)
                    stack.append(new_node)
                
                trav()
        trav()
        # Reverse the order
        sorted_order.reverse()
        return sorted_order
    
    def dijkstra(self, start_node: Node):
        """
        Dijkstra to find the shortest paths between each nodes

        Args:
            start_node: Node type class (default = None)

        Returns:
            dist: Dict type, returns node name as key and distance as item
        """
        # Queue containing the nodes except the start node
        queue = list(self.node_dict.values())
        # Shortest path dict
        dist = dict()
        # Set all values in the dist dict to inf
        for node in queue:
            dist[node.name] = np.inf
        # Start node set to zero
        dist[start_node.name] = 0.0

        def find_next_node(connections: Node):
            distances = []
            for connection in connections:
                distances.append(dist[connection.name])
            min_index = min(range(len(distances)), key=lambda i: distances[i])
            return connections[min_index]

        def calc_path_sum(node: Node):
            queue.remove(node)
            while queue:
                node_dist = dist[node.name]
                for connection in node.connections:
                    path_sum = node_dist + connection.value
                    if path_sum < dist[connection.name]:
                        dist[connection.name] = path_sum
                
                shortest_node = find_next_node(queue)
                calc_path_sum(shortest_node)
            return
            
        calc_path_sum(start_node)
        return dist


if __name__ == "__main__":
    node_a = Node(0)

    node_b = Node(1)

    node_a.add_connection(node_b)
    print(node_a.connections[0].value)
    print(node_b.connections[0].value)

    # Adjacency matrix
    no_nodes = 10
    A = np.ones((no_nodes, no_nodes))
    np.fill_diagonal(A, 0.0)

    A[no_nodes - 1, 0] *= 0.0
    A[0, no_nodes - 1] *= 0.0

    for k in range(no_nodes - 1):
        A[2, k] *= 0.0
        A[k, 2] *= 0.0

    values = np.ones(no_nodes) # list(range(0, no_nodes, 1))
    values[-1] = 10.0

    graph = Graph(A, values)

    print(graph)
    print(graph.adjacency_mat)

    print(np.linalg.eigvals(graph.adjacency_mat))

    shortest_path = graph.dijkstra(graph.node_dict[graph.node_names[0]])
    print(shortest_path.items())
