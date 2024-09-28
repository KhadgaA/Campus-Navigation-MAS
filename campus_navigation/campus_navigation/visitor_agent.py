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
    visitor_id_counter = 1

    def spawn_visitor():
        nonlocal visitor_id_counter
        visitor_agent = VisitorAgent(f'visitor_{visitor_id_counter}')
        visitor_id_counter += 1
        buildings = ['Building A', 'Building B', 'Building C']
        building = random.choice(buildings)
        visitor_agent.send_navigation_request(building)
        visitor_agent.move_to_building(building)
        visitor_agent.destroy_node()

    # Spawn visitors at random intervals
    while rclpy.ok():
        spawn_visitor()
        time.sleep(random.randint(5, 15))  # Random interval between 5 and 15 seconds

    rclpy.shutdown()

if __name__ == '__main__':
    main()
