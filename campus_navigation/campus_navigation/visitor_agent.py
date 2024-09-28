import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest
import time
import random

class VisitorAgent(Node):
    def __init__(self, visitor_id):
        super().__init__(f'visitor_agent_{visitor_id}')
        self.publisher_ = self.create_publisher(NavigationRequest, 'navigation_request', 10)
        self.visitor_id = visitor_id
        self.current_location = 'Entrance'

    def send_navigation_request(self, building_id):
        msg = NavigationRequest()
        msg.visitor_id = self.visitor_id
        msg.building_id = building_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published navigation request: {msg}')

    def move_to_building(self, building_id):
        self.current_location = building_id
        self.get_logger().info(f'Visitor {self.visitor_id} moved to {building_id}')
        time.sleep(2)  # Simulate movement delay

def main(args=None):
    rclpy.init(args=args)
    visitor_agents = [VisitorAgent(f'visitor_{i}') for i in range(1, 4)]

    buildings = ['Building A', 'Building B', 'Building C']
    for agent in visitor_agents:
        building = random.choice(buildings)
        agent.send_navigation_request(building)
        agent.move_to_building(building)
        time.sleep(random.randint(1, 5))  # Simulate random meeting durations

    rclpy.spin(visitor_agents[0])  # Spin one of the agents to keep the node running
    for agent in visitor_agents:
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
