import networkx as nx
import matplotlib.pyplot as plt

def create_campus_graph():
    G = nx.DiGraph()

    # Add nodes (locations)
    G.add_node("Entrance")
    G.add_node("Building A")
    G.add_node("Building B")
    G.add_node("Building C")

    # Add edges (paths)
    G.add_edge("Entrance", "Building A", weight=5)
    G.add_edge("Entrance", "Building B", weight=10)
    G.add_edge("Building A", "Building C", weight=7)
    G.add_edge("Building B", "Building C", weight=3)

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
