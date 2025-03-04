#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import time

class MapClient(Node):
    def __init__(self):
        super().__init__('map_client')

        # Create a client for the GetMap service
        self.client = self.create_client(GetMap, '/map_server/map')

        # Create a publisher to publish the map
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Wait until the service is available before proceeding
        self.wait_for_service()

        # Call the service to get the map
        self.map_data = None
        self.call_map_service()

        # Timer to publish the map every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_map_periodically)  # Publish every 5 seconds

    def wait_for_service(self):
        """Wait indefinitely until the service is available."""
        self.get_logger().info("Waiting for the service /map_server/map...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available yet. Retrying...")
        self.get_logger().info("Service found!")

    def call_map_service(self):
        """Call the service to get the map."""
        request = GetMap.Request()
        future = self.client.call_async(request)

        # Wait for the service response indefinitely
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info("Map received.")
                self.map_data = response.map  # Store the map for publishing
            except Exception as e:
                self.get_logger().error(f"Error while calling the service: {e}")
        else:
            self.get_logger().error("Timeout waiting for the service response.")

    def publish_map(self):
        """Publish the map to the topic topic every 5 seconds."""
        if self.map_data:
            self.get_logger().info("Publishing the map")
            self.publisher.publish(self.map_data)
        else:
            self.get_logger().error("Map not received, unable to publish.")

    def publish_map_periodically(self):
        """This function is called every 5 seconds to publish the map."""
        self.publish_map()

def main(args=None):
    rclpy.init(args=args)

    # Create the MapClient to handle the map retrieval and publishing
    map_client = MapClient()

    # Spin to keep the script running
    rclpy.spin(map_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
