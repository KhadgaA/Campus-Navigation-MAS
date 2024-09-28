import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest, NavigationResponse, AgentMovement
import time
import networkx as nx

class CIAgent(Node):
    def __init__(self, agent_id):
        super().__init__(f'ci_agent_{agent_id}')
        self.publisher_ = self.create_publisher(NavigationRequest, 'navigation_request', 10)
        self.movement_publisher_ = self.create_publisher(AgentMovement, 'agent_movement', 10)
        self.subscription = self.create_subscription(
            NavigationResponse,
            'navigation_response',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_location = 'Entrance'
        self.visitors_entertained = 0
        self.violation_events = 0
        self.agent_id = agent_id
        self.visitor_queue = []
        self.campus_graph = self.create_campus_graph()

    def create_campus_graph(self):
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

    def listener_callback(self, msg):
        self.get_logger().info(f'Received navigation response: {msg}')
        # Publish movement information
        self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)
        self.visitors_entertained += 1

    def send_navigation_request(self, visitor_id, building_id):
        path = nx.shortest_path(self.campus_graph, source=self.current_location, target=building_id)
        for i in range(len(path) - 1):
            msg = NavigationRequest()
            msg.ci_agent_id = str(self.agent_id)  # Convert agent_id to string
            msg.visitor_id = visitor_id
            msg.building_id = path[i + 1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published navigation request: {msg}')
            self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)

    def move_agent(self, ci_agent_id, visitor_id, building_id):
        # Simulate movement by updating the current location
        self.current_location = building_id
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = self.current_location
        movement_msg.to_location = building_id
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  # Simulate movement delay

    def log_performance(self):
        self.get_logger().info(f'CI Agent Performance: Visitors entertained = {self.visitors_entertained}, Violation events = {self.violation_events}')

    def handle_visitor(self, visitor_id, building_id):
        self.visitor_queue.append((visitor_id, building_id))
        if len(self.visitor_queue) == 1:  # If this is the only visitor in the queue
            self.send_navigation_request(visitor_id, building_id)

def main(args=None):
    rclpy.init(args=args)
    ci_agents = [CIAgent(i) for i in range(1, 4)]  # Start with 3 CI agents

    rclpy.spin(ci_agents[0])  # Spin one of the agents to keep the node running
    for agent in ci_agents:
        agent.log_performance()
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
