import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest, NavigationResponse, OOSNotification, AgentMovement
import time
import random
import networkx as nx

class BIAgent(Node):
    def __init__(self, agent_id, building_id):
        super().__init__(f'bi_agent_{agent_id}_{building_id}')
        self.subscription = self.create_subscription(
            NavigationRequest,
            'navigation_request',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(NavigationResponse, 'navigation_response', 10)
        self.oos_publisher_ = self.create_publisher(OOSNotification, 'oos_notification', 10)
        self.movement_publisher_ = self.create_publisher(AgentMovement, 'agent_movement', 10)
        self.subscription  
        self.ci_agents_guided = 0
        self.violation_events = 0
        self.oos_duration = 0
        self.oos_start_time = None
        self.agent_id = agent_id
        self.building_id = building_id
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
        if self.is_out_of_service():
            self.get_logger().info(f'BI Agent {self.get_name()} is out of service. Cannot handle request.')
            return

        if msg.building_id != self.building_id:
            self.get_logger().info(f'BI Agent {self.get_name()} cannot handle request for {msg.building_id}.')
            return

        self.get_logger().info(f'Received navigation request: {msg}')
        response = NavigationResponse()
        response.ci_agent_id = msg.ci_agent_id
        response.visitor_id = msg.visitor_id
        response.building_id = msg.building_id
        response.path = ['path_segment_1', 'path_segment_2']
        response.access_granted = True
        self.publisher_.publish(response)
        self.get_logger().info(f'Published navigation response: {response}')

        
        self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)
        self.ci_agents_guided += 1

    def send_oos_notification(self, duration):
        self.oos_duration = duration
        self.oos_start_time = time.time()
        msg = OOSNotification()
        msg.bi_agent_id = self.get_name()
        msg.duration = duration
        self.oos_publisher_.publish(msg)
        self.get_logger().info(f'Published OOS notification: {msg}')

    def move_agent(self, ci_agent_id, visitor_id, building_id):
        
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = building_id
        movement_msg.to_location = building_id
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  

        
        self.explore_building(ci_agent_id, visitor_id, building_id)

    def explore_building(self, ci_agent_id, visitor_id, building_id):
        self.get_logger().info(f'BI Agent is exploring {building_id} with {visitor_id}')
        time.sleep(5)  

        
        self.return_visitor(ci_agent_id, visitor_id, building_id)

    def return_visitor(self, ci_agent_id, visitor_id, building_id):
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = building_id
        movement_msg.to_location = 'Entrance'
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  

    def is_out_of_service(self):
        if self.oos_start_time is None:
            return False
        current_time = time.time()
        return current_time - self.oos_start_time < self.oos_duration

    def log_performance(self):
        self.get_logger().info(f'BI Agent Performance: CI agents guided = {self.ci_agents_guided}, Violation events = {self.violation_events}')

def main(args=None):
    rclpy.init(args=args)
    bi_agents = [BIAgent(i, building) for i in range(1, 2) for building in ['A', 'B', 'C', 'D', 'E', 'Library', 'Cafeteria', 'Gym']]  

    
    
    
    
    

    rclpy.spin(bi_agents[0])  
    for agent in bi_agents:
        agent.log_performance()
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
