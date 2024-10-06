import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest, NavigationResponse, OOSNotification, AgentMovement
import networkx as nx
import matplotlib.pyplot as plt

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.subscription_request = self.create_subscription(
            NavigationRequest,
            'navigation_request',
            self.request_callback,
            10)
        self.subscription_response = self.create_subscription(
            NavigationResponse,
            'navigation_response',
            self.response_callback,
            10)
        self.subscription_oos = self.create_subscription(
            OOSNotification,
            'oos_notification',
            self.oos_callback,
            10)
        self.subscription_movement = self.create_subscription(
            AgentMovement,
            'agent_movement',
            self.movement_callback,
            10)
        self.subscription_request  
        self.subscription_response  
        self.subscription_oos  
        self.subscription_movement  

        self.campus_graph = self.create_campus_graph()
        self.pos = nx.spring_layout(self.campus_graph)
        self.agent_positions = {}
        self.draw_graph(self.campus_graph)

        
        self.timer = self.create_timer(1.0, self.timer_callback)

    def request_callback(self, msg):
        self.get_logger().info(f'Received navigation request: {msg}')
        
        self.update_graph(msg.ci_agent_id, msg.visitor_id, msg.building_id, 'request')

    def response_callback(self, msg):
        self.get_logger().info(f'Received navigation response: {msg}')
        
        self.update_graph(msg.ci_agent_id, msg.visitor_id, msg.building_id, 'response')

    def oos_callback(self, msg):
        self.get_logger().info(f'Received OOS notification: {msg}')
        
        self.update_graph(msg.bi_agent_id, None, None, 'oos')

    def movement_callback(self, msg):
        self.get_logger().info(f'Received agent movement: {msg}')
        
        self.update_graph(msg.agent_id, msg.visitor_id, msg.to_location, 'movement')

    def create_campus_graph(self):
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

    def draw_graph(self, G):
        plt.clf()  
        nx.draw(G, self.pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, font_weight='bold')
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, self.pos, edge_labels=edge_labels)

        
        for agent_id, position in self.agent_positions.items():
            if 'ci_agent' in agent_id:
                color = 'green'
            elif 'bi_agent' in agent_id:
                color = 'red'
            else:
                color = 'yellow'
            if position in self.pos:
                nx.draw_networkx_nodes(G, self.pos, nodelist=[position], node_color=color, node_size=300)

        plt.ion()  
        plt.draw()  
        plt.pause(0.01)  

    def update_graph(self, agent_id, visitor_id, building_id, event_type):
        
        if event_type == 'request':
            self.get_logger().info(f'CI Agent {agent_id} is escorting Visitor {visitor_id} to Building {building_id}')
            self.agent_positions[agent_id] = building_id
        elif event_type == 'response':
            self.get_logger().info(f'BI Agent responded to CI Agent {agent_id} for Visitor {visitor_id} in Building {building_id}')
            self.agent_positions[agent_id] = building_id
        elif event_type == 'oos':
            self.get_logger().info(f'BI Agent {agent_id} is out of service')
            self.agent_positions[agent_id] = 'Out of Service'
        elif event_type == 'movement':
            self.get_logger().info(f'Agent {agent_id} is moving Visitor {visitor_id} to Building {building_id}')
            self.agent_positions[agent_id] = building_id
            if visitor_id:
                self.agent_positions[visitor_id] = building_id

        self.draw_graph(self.campus_graph)  

    def timer_callback(self):
        self.draw_graph(self.campus_graph)  

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
