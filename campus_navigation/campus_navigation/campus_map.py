import networkx as nx
import matplotlib.pyplot as plt

def create_campus_graph():
    G = nx.Graph()

    # Add nodes (locations)
    G.add_node("Entrance")
    G.add_node("Building A")
    G.add_node("Building B")
    G.add_node("Building C")
    G.add_node("Building D")
    G.add_node("Building E")
    G.add_node("Library")
    G.add_node("Cafeteria")
    G.add_node("Gym")

    # Add edges (paths)
    G.add_edge("Entrance", "Building A", weight=5)
    G.add_edge("Entrance", "Building B", weight=10)
    G.add_edge("Building A", "Building C", weight=7)
    G.add_edge("Building B", "Building C", weight=3)
    G.add_edge("Building C", "Building D", weight=8)
    G.add_edge("Building D", "Building E", weight=6)
    G.add_edge("Building E", "Library", weight=4)
    G.add_edge("Library", "Cafeteria", weight=2)
    G.add_edge("Cafeteria", "Gym", weight=9)
    G.add_edge("Gym", "Building A", weight=12)

    return G

def draw_graph(G):
    pos = nx.spring_layout(G)
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, font_weight='bold')
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    plt.show()

if __name__ == "__main__":
    G = create_campus_graph()
    draw_graph(G)
