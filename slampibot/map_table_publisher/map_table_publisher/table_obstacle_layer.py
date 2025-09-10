#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_costmap_2d.costmap_2d import Costmap2D
from nav2_costmap_2d.costmap_2d_ros import Costmap2DROS
from geometry_msgs.msg import PolygonStamped
import numpy as np

class TableObstacleLayer(Node):
    def __init__(self):
        super().__init__('table_obstacle_layer')
        
        # Parameters
        self.declare_parameter('enabled', True)
        self.declare_parameter('polygon_topic', '/table_obstacle')
        
        # Get parameters
        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        polygon_topic = self.get_parameter('polygon_topic').get_parameter_value().string_value
        
        # Subscriber
        self.polygon_sub = self.create_subscription(
            PolygonStamped,
            polygon_topic,
            self.polygon_callback,
            10
        )
        
        self.get_logger().info(f'Table Obstacle Layer started, subscribing to {polygon_topic}')
        
    def polygon_callback(self, msg):
        if not self.enabled:
            return
            
        self.get_logger().info(f'Received polygon with {len(msg.polygon.points)} points')
        # Here you would implement the logic to add the polygon to the costmap
        # This is a simplified version - in practice, you would need to interface
        # with the costmap layers directly
        
def main(args=None):
    rclpy.init(args=args)
    node = TableObstacleLayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()