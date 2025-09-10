
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import math
import numpy as np

class SmartPolygonToPointCloudNode(Node):
    def __init__(self):
        super().__init__('polygon_to_pointcloud_node')
        
        # Store the latest merged polygon
        self.latest_polygon = None
        self.subscription = self.create_subscription(
            PolygonStamped,
            '/merged_table_obstacles',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(PointCloud2, '/obstacle_point_cloud', 10)
        self.publish_timer = self.create_timer(0.2, self.publish_combined_cloud)

        self.get_logger().info('Smart Polygon to PointCloud2 converter node started.')

    def listener_callback(self, msg):
        # Store the latest merged polygon
        self.latest_polygon = msg
        self.get_logger().debug(f"Received merged polygon with {len(msg.polygon.points)} points")

    def publish_combined_cloud(self):
        if not self.latest_polygon:
            return

        polygon_points = self.latest_polygon.polygon.points
        if len(polygon_points) < 3:
            return

        all_points = []
        current_time = self.get_clock().now().to_msg()
        map_frame_id = "map"

        # Process the polygon points
        # We expect each table to be represented by 4 consecutive points
        num_tables = len(polygon_points) // 4
        self.get_logger().info(f"Processing {num_tables} tables from {len(polygon_points)} points")
        
        for table_idx in range(num_tables):
            # Extract 4 points for this table
            start_idx = table_idx * 4
            table_points = polygon_points[start_idx:start_idx + 4]
            
            # Verify we have 4 points
            if len(table_points) != 4:
                self.get_logger().warn(f"Table {table_idx} has {len(table_points)} points instead of 4, skipping")
                continue
                
            # Log the table position (using the first point as representative)
            self.get_logger().info(f"Table {table_idx+1} at position ({table_points[0].x:.2f}, {table_points[0].y:.2f})")
            
            # Generate points along the table edges
            density = 0.05  # 5cm between points
            for i in range(len(table_points)):
                p1 = table_points[i]
                p2 = table_points[(i + 1) % len(table_points)]
                
                # Calculate distance between points
                dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
                # Calculate number of points based on density
                num_points = max(1, int(dist / density))

                # Generate points along the edge
                for j in range(num_points + 1):
                    ratio = float(j) / num_points if num_points > 0 else 0
                    x = p1.x + ratio * (p2.x - p1.x)
                    y = p1.y + ratio * (p2.y - p1.y)
                    z = 0.0  # Set Z to 0.0 for floor level
                    all_points.append([x, y, z])

        if not all_points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = map_frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        point_step = 12
        row_step = point_step * len(all_points)
        
        # Create data using numpy for better performance and compatibility
        points_array = np.array(all_points, dtype=np.float32)
        cloud_data = points_array.tobytes()
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(all_points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.fields = fields
        cloud_msg.point_step = point_step
        cloud_msg.row_step = row_step
        cloud_msg.data = cloud_data

        self.publisher.publish(cloud_msg)
        self.get_logger().info(f"Published a combined cloud with {len(all_points)} points representing {num_tables} tables.")

def main(args=None):
    rclpy.init(args=args)
    node = SmartPolygonToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
