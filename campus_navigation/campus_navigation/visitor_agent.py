import rclpy
from rclpy.node import Node
from campus_navigation_msgs.msg import NavigationRequest
import time
import random
import networkx as nx
import asyncio

class VisitorAgent(Node):
    def __init__(self, visitor_id):
        super().__init__(f'visitor_agent_{visitor_id}')
        self.publisher_ = self.create_publisher(NavigationRequest, 'navigation_request', 10)
        self.visitor_id = visitor_id
        self.current_location = 'Entrance'
        self.buildings_to_explore = [
            "Entrance", "Building A", "Building B", "Building C",
            "Building D", "Building E", "Library", "Cafeteria", "Gym",
            "Lecture Hall", "Research Center", "Dormitory", "Auditorium"
        ]
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

    def send_navigation_request(self, building_id):
        path = nx.shortest_path(self.campus_graph, source=self.current_location, target=building_id)
        for i in range(len(path) - 1):
            msg = NavigationRequest()
            msg.visitor_id = self.visitor_id    
            msg.building_id = path[i + 1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published navigation request: {msg}')
            self.move_to_building(path[i + 1])
            time.sleep(1)

    def move_to_building(self, building_id):
        self.current_location = building_id
        self.get_logger().info(f'Visitor {self.visitor_id} moved to {building_id}')
        time.sleep(2)  

    def explore_buildings(self):
        
        building = random.choice(self.buildings_to_explore)
        self.get_logger().info(f'Visitor to explore: {building}')
        self.send_navigation_request(building)
        time.sleep(random.randint(5, 10))  
        
        self.send_navigation_request('Entrance')

# def main(args=None):
#     rclpy.init(args=args)
#     visitor_id_counter = 1

#     def spawn_visitor():
#         nonlocal visitor_id_counter
#         visitor_agent = VisitorAgent(f'visitor_{visitor_id_counter}')
#         visitor_id_counter += 1
#         visitor_agent.explore_buildings()
#         visitor_agent.destroy_node()

#     # Spawn visitors at random intervals
#     while rclpy.ok():
#         spawn_visitor()
#         time.sleep(random.randint(5, 15))  # Random interval between 5 and 15 seconds

#     rclpy.shutdown()
async def spawn_visitor(visitor_id):
    """Function to create and manage a visitor asynchronously."""
    visitor_agent = VisitorAgent(f'visitor_{visitor_id}')
    await visitor_agent.explore_buildings()  # Assuming this function can be awaited or should be async
    visitor_agent.destroy_node()

async def main():
    rclpy.init(args=None)
    visitor_id_counter = 1

    async def spawn_visitors():
        nonlocal visitor_id_counter
        visitor_agent = VisitorAgent(f'visitor_{visitor_id_counter}')
        visitor_id_counter += 1
        visitor_agent.explore_buildings()
        visitor_agent.destroy_node()

    
    while rclpy.ok():
        spawn_visitor()
        time.sleep(random.randint(5, 15))  

    rclpy.shutdown()
if __name__ == '__main__':
    asyncio.run(main())
