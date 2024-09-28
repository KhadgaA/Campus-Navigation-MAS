import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest
import time
import random
import networkx as nx

class VisitorAgent(Node):
    def __init__(self, visitor_id):
        super().__init__(f'visitor_agent_{visitor_id}')
        self.publisher_ = self.create_publisher(NavigationRequest, 'navigation_request', 10)
        self.visitor_id = visitor_id
        self.current_location = 'Entrance'
        self.buildings_to_explore = ['Building A', 'Building B', 'Building C']
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

    def send_navigation_request(self, building_id):
        path = nx.shortest_path(self.campus_graph, source=self.current_location, target=building_id)
        for i in range(len(path) - 1):
            msg = NavigationRequest()
            msg.visitor_id = self.visitor_id
            msg.building_id = path[i + 1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published navigation request: {msg}')
            self.move_to_building(path[i + 1])

    def move_to_building(self, building_id):
        self.current_location = building_id
        self.get_logger().info(f'Visitor {self.visitor_id} moved to {building_id}')
        time.sleep(2)  # Simulate movement delay

    def explore_buildings(self):
        for building in self.buildings_to_explore:
            self.send_navigation_request(building)
            time.sleep(random.randint(5, 10))  # Simulate exploring time

        # After exploring all buildings, request to be escorted back to the entrance
        self.send_navigation_request('Entrance')

def main(args=None):
    rclpy.init(args=args)
    visitor_id_counter = 1

    def spawn_visitor():
        nonlocal visitor_id_counter
        visitor_agent = VisitorAgent(f'visitor_{visitor_id_counter}')
        visitor_id_counter += 1
        visitor_agent.explore_buildings()
        visitor_agent.destroy_node()

    # Spawn visitors at random intervals
    while rclpy.ok():
        spawn_visitor()
        time.sleep(random.randint(5, 15))  # Random interval between 5 and 15 seconds

    rclpy.shutdown()

if __name__ == '__main__':
    main()
