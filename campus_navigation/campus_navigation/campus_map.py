import networkx as nx
import matplotlib.pyplot as plt

def create_campus_graph():
    G = nx.Graph()  

    
    nodes = [
        "Entrance", "Building A", "Building B", "Building C",
        "Building D", "Building E", "Library", "Cafeteria", "Gym",
        "Lecture Hall", "Research Center", "Dormitory", "Auditorium"
    ]
    G.add_nodes_from(nodes)

    
    edges = [
        ("Entrance", "Building A", 5),
        ("Entrance", "Building B", 10),
        ("Building A", "Lecture Hall", 3),
        ("Building A", "Building C", 7),
        ("Building B", "Building C", 3),
        ("Building C", "Building D", 8),
        ("Building D", "Building E", 6),
        ("Building E", "Library", 4),
        ("Library", "Cafeteria", 2),
        ("Cafeteria", "Gym", 9),
        ("Gym", "Building A", 12),
        ("Building C", "Research Center", 5),
        ("Building D", "Auditorium", 4),
        ("Research Center", "Building E", 7),
        ("Dormitory", "Building B", 10),
        ("Auditorium", "Entrance", 15),
    ]
    
    
    G.add_weighted_edges_from(edges)

    
    for node in G.nodes():
        if G.degree(node) == 1:  
            neighbors = list(G.neighbors(node))
            for neighbor in neighbors:
                if not G.has_edge(neighbor, node):  
                    G.add_edge(neighbor, node, weight=G[node][neighbor]['weight'])  

    return G

def draw_graph(G):
    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, font_weight='bold', arrows=True)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    plt.title("Campus Navigation Graph")
    plt.show()

if __name__ == "__main__":
    G = create_campus_graph()
    draw_graph(G)
