from Graph import Graph, Node

import numpy as np


class WordLadder:
    """
    Creates a word ladder and finds the shorest path between start word 
    and all others
    """
    def __init__(self, word_list) -> None:
        self.graph = self.create_graph(word_list)

    def create_adjacency(self, words: list) -> np.array:
        """
        Creates the adjacency matrix from the word list

        Args: 
            words: List type
        
        Returns:
            numpy array
        """
        # Number of words in the list
        no_words = len(words)
        # Create Adjacency matrix for graph
        A = np.zeros([no_words, no_words])
        # Iterate through each word and 
        for k in range(no_words):
            word = words[k]
            # Look at the right many words in words
            for j in range(k + 1, no_words, 1):
                # Test to see if one letter is different
                word_bool = np.invert(np.compare_chararrays(list(word), list(words[j]), "==", True))

                if word_bool.sum() == 1:
                    A[j, k] = 1.0
        A += A.transpose()
        return A
    
    def create_graph(self, words: list) -> Graph:
        """
        Creates graph from the word list

        Args: 
            words: List type
        
        Returns:
            Graph
        """
        A = self.create_adjacency(words)
        values = [1.0] * len(words)   
        return Graph(A, node_values=values, node_names=words)
    
    def shortest_path(self, start_word) -> dict:
        """
        Finds shortest path from start word to all other words and 
        stores result in a dictionary

        Args: 
            words: List type
        
        Returns:
            dict: finding distance from start word to all other words
        """
        start_node = self.graph.node_dict[start_word]
        return self.graph.dijkstra(start_node)

    
if __name__ == "__main__":

    word_list = ["hot","dot","dog","lot","log","cog","mag","lag","mat","cat"]
    print(len(word_list))
    word_ladder = WordLadder(word_list)

    A = word_ladder.create_adjacency(word_list)
    print(A)

    shortest_path = word_ladder.shortest_path(word_list[0])
    print(shortest_path)
            