#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from simulation_msgs.srv import PausePlay
import time

class PausePlayClient(Node):
    def __init__(self):
        super().__init__('pause_play_client')
        self.cli = self.create_client(PausePlay, '/unity/pause_play')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /unity/pause_play non disponible, attente...')
        self.pause = False  # État initial
        self.timer = self.create_timer(0.5, self.timer_callback)  # toutes les 0.5 sec

    def timer_callback(self):
        self.pause = not self.pause
        self.get_logger().info(f'Envoi de la demande pause={self.pause}')

        # Appeler le service sans argument car Trigger.Request est vide
        req = PausePlay.Request()
        req.play = not self.pause

        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Pause/Play effectué avec succès')
            else:
                self.get_logger().warn(f'Échec pause/play: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Erreur appel service : {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PausePlayClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
