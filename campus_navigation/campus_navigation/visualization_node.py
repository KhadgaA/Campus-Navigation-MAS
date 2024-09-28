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
        self.subscription_request  # prevent unused variable warning
        self.subscription_response  # prevent unused variable warning
        self.subscription_oos  # prevent unused variable warning
        self.subscription_movement  # prevent unused variable warning

        self.campus_graph = self.create_campus_graph()
        self.pos = nx.spring_layout(self.campus_graph)
        self.agent_positions = {}
        self.draw_graph(self.campus_graph)

        # Timer to periodically redraw the graph
        self.timer = self.create_timer(1.0, self.timer_callback)

    def request_callback(self, msg):
        self.get_logger().info(f'Received navigation request: {msg}')
        # Update the graph with the request information
        self.update_graph(msg.ci_agent_id, msg.visitor_id, msg.building_id, 'request')

    def response_callback(self, msg):
        self.get_logger().info(f'Received navigation response: {msg}')
        # Update the graph with the response information
        self.update_graph(msg.ci_agent_id, msg.visitor_id, msg.building_id, 'response')

    def oos_callback(self, msg):
        self.get_logger().info(f'Received OOS notification: {msg}')
        # Update the graph with the OOS information
        self.update_graph(msg.bi_agent_id, None, None, 'oos')

    def movement_callback(self, msg):
        self.get_logger().info(f'Received agent movement: {msg}')
        # Update the graph with the movement information
        self.update_graph(msg.agent_id, msg.visitor_id, msg.to_location, 'movement')

    def create_campus_graph(self):
        G = nx.DiGraph()

        # Add nodes (locations)
        G.add_node("Entrance")
        G.add_node("Building A")
        G.add_node("Building B")
        G.add_node("Building C")
        G.add_node("Out of Service")  # Add a node for the "Out of Service" state

        # Add edges (paths)
        G.add_edge("Entrance", "Building A", weight=5)
        G.add_edge("Entrance", "Building B", weight=10)
        G.add_edge("Building A", "Building C", weight=7)
        G.add_edge("Building B", "Building C", weight=3)

        return G

    def draw_graph(self, G):
        plt.clf()  # Clear the current figure
        nx.draw(G, self.pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10, font_weight='bold')
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, self.pos, edge_labels=edge_labels)

        # Draw agents and visitors
        for agent_id, position in self.agent_positions.items():
            if 'ci_agent' in agent_id:
                color = 'green'
            elif 'bi_agent' in agent_id:
                color = 'red'
            else:
                color = 'yellow'
            if position in self.pos:
                nx.draw_networkx_nodes(G, self.pos, nodelist=[position], node_color=color, node_size=300)

        plt.ion()  # Turn on interactive mode
        plt.draw()  # Draw the graph
        plt.pause(0.01)  # Pause to update the graph

    def update_graph(self, agent_id, visitor_id, building_id, event_type):
        # Update the graph based on the event type
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

        self.draw_graph(self.campus_graph)  # Redraw the graph

    def timer_callback(self):
        self.draw_graph(self.campus_graph)  # Periodically redraw the graph

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
