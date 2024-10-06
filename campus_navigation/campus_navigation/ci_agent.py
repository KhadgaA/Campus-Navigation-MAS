import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest, NavigationResponse, AgentMovement
import time
import networkx as nx
import asyncio
from rclpy.executors import MultiThreadedExecutor

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
        self.subscription  
        self.current_location = 'Entrance'
        self.visitors_entertained = 0
        self.violation_events = 0
        self.agent_id = agent_id
        self.visitor_queue = []
        self.campus_graph = self.create_campus_graph()

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

    def listener_callback(self, msg):
        self.get_logger().info(f'Received navigation response: {msg}')
        
        self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)
        self.visitors_entertained += 1

    def send_navigation_request(self, visitor_id, building_id):
        path = nx.shortest_path(self.campus_graph, source=self.current_location, target=building_id)
        for i in range(len(path) - 1):
            msg = NavigationRequest()
            msg.ci_agent_id = str(self.agent_id)  
            msg.visitor_id = visitor_id
            msg.building_id = path[i + 1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published navigation request: {msg}')
            self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)

    def move_agent(self, ci_agent_id, visitor_id, building_id):
        
        self.current_location = building_id
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = self.current_location
        movement_msg.to_location = building_id
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  

    def log_performance(self):
        self.get_logger().info(f'CI Agent Performance: Visitors entertained = {self.visitors_entertained}, Violation events = {self.violation_events}')

    def handle_visitor(self, visitor_id, building_id):
        self.visitor_queue.append((visitor_id, building_id))
        if len(self.visitor_queue) == 1:  
            self.send_navigation_request(visitor_id, building_id)

def main(args=None):
    rclpy.init(args=args)
    ci_agents = [CIAgent(i) for i in range(1, 4)]  

    rclpy.spin(ci_agents[0])  
    for agent in ci_agents:
        agent.log_performance()
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init(args=None)  # Initialize ROS 2 nodes outside asyncio
    loop = asyncio.get_event_loop()
    
    try:
        loop.run_until_complete(main())  # Use asyncio's event loop
    finally:
        loop.close()  # Ensure cleanup