import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest, NavigationResponse, AgentMovement
import time

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

    def listener_callback(self, msg):
        self.get_logger().info(f'Received navigation response: {msg}')
        # Publish movement information
        self.move_agent(msg.ci_agent_id, msg.visitor_id, msg.building_id)
        self.visitors_entertained += 1

    def send_navigation_request(self, visitor_id, building_id):
        msg = NavigationRequest()
        msg.ci_agent_id = str(self.agent_id)  # Convert agent_id to string
        msg.visitor_id = visitor_id
        msg.building_id = building_id
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published navigation request: {msg}')

    def move_agent(self, ci_agent_id, visitor_id, building_id):
        # Simulate movement by updating the current location
        self.current_location = building_id
        movement_msg = AgentMovement()
        movement_msg.agent_id = ci_agent_id
        movement_msg.visitor_id = visitor_id
        movement_msg.from_location = 'Entrance'
        movement_msg.to_location = building_id
        self.movement_publisher_.publish(movement_msg)
        self.get_logger().info(f'Published agent movement: {movement_msg}')
        time.sleep(2)  # Simulate movement delay

    def log_performance(self):
        self.get_logger().info(f'CI Agent Performance: Visitors entertained = {self.visitors_entertained}, Violation events = {self.violation_events}')

def main(args=None):
    rclpy.init(args=args)
    ci_agents = [CIAgent(i) for i in range(1, 4)]

    rclpy.spin(ci_agents[0])  # Spin one of the agents to keep the node running
    for agent in ci_agents:
        agent.log_performance()
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
