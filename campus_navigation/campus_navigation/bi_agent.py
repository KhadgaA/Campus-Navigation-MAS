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
        self.subscription  # prevent unused variable warning
        self.ci_agents_guided = 0
        self.violation_events = 0
        self.oos_duration = 0
        self.oos_start_time = None
        self.agent_id = agent_id
        self.building_id = building_id
        self.campus_graph = self.create_campus_graph()

    def create_campus_graph(self):
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

    def listener_callback(self, msg):
        if self.is_out_of_service():
            self.get_logger().info(f'BI Agent {self.get_name()} is out of service. Cannot handle request.')
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

        # Publish movement information
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
        # Simulate movement by updating the current location
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = 'Entrance'
        movement_msg.to_location = building_id
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  # Simulate movement delay

        # Simulate exploring the building
        self.explore_building(ci_agent_id, visitor_id, building_id)

    def explore_building(self, ci_agent_id, visitor_id, building_id):
        self.get_logger().info(f'BI Agent is exploring {building_id} with {visitor_id}')
        time.sleep(5)  # Simulate exploring time

        # Return the visitor to the CI agent
        self.return_visitor(ci_agent_id, visitor_id, building_id)

    def return_visitor(self, ci_agent_id, visitor_id, building_id):
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = building_id
        movement_msg.to_location = 'Entrance'
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  # Simulate movement delay

    def is_out_of_service(self):
        if self.oos_start_time is None:
            return False
        current_time = time.time()
        return current_time - self.oos_start_time < self.oos_duration

    def log_performance(self):
        self.get_logger().info(f'BI Agent Performance: CI agents guided = {self.ci_agents_guided}, Violation events = {self.violation_events}')

def main(args=None):
    rclpy.init(args=args)
    bi_agents = [BIAgent(i, building) for i in range(1, 3) for building in ['A', 'B', 'C', 'D', 'E', 'Library', 'Cafeteria', 'Gym']]  # Start with 2 BI agents for each building

    # Simulate random OOS notifications with shorter durations
    for agent in bi_agents:
        agent.send_oos_notification(random.randint(5, 10))
        time.sleep(random.randint(1, 5))  # Wait for some time before sending another OOS notification

    rclpy.spin(bi_agents[0])  # Spin one of the agents to keep the node running
    for agent in bi_agents:
        agent.log_performance()
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
