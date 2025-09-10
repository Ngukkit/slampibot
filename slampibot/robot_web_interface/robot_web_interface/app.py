import rclpy
from rclpy.node import Node
from robot_commander_interfaces.srv import SendWaypoint
from flask import Flask, render_template, request, jsonify
import threading
import time
import os

# Try to find the templates directory in multiple possible locations
# This approach works for both source and installed packages
template_dir = None

# Get the installation prefix (if available)
install_prefix = os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0]

# List of possible template directories to check
possible_paths = [
    # Source location (for development)
    os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'templates'),
    # Standard install location
    os.path.join(install_prefix, 'share', 'robot_web_interface', 'templates') if install_prefix else None,
    # Alternative install location relative to this file
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), 'share', 'robot_web_interface', 'templates'),
]

# Filter out None values
possible_paths = [path for path in possible_paths if path is not None]

# Check each path for the index.html file
for path in possible_paths:
    if os.path.exists(os.path.join(path, 'index.html')):
        template_dir = path
        break

# If we still haven't found it, try a more general search
if template_dir is None:
    # Try to find templates directory relative to known ROS2 paths
    import ament_index_python
    try:
        package_share_dir = ament_index_python.get_package_share_directory('robot_web_interface')
        template_path = os.path.join(package_share_dir, 'templates')
        if os.path.exists(os.path.join(template_path, 'index.html')):
            template_dir = template_path
    except ament_index_python.PackageNotFoundError:
        pass

if template_dir is None:
    raise FileNotFoundError("Could not find the templates directory with index.html. Checked paths: " + ", ".join(possible_paths))

# Create the Flask app with the correct template directory
app = Flask(__name__, template_folder=template_dir)

# Create the Flask app with the correct template directory
app = Flask(__name__, template_folder=template_dir)

# ROS 2 Node for service client
class WebCommanderClient(Node):
    def __init__(self):
        super().__init__('web_commander_client')
        self.cli = self.create_client(SendWaypoint, 'send_waypoint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SendWaypoint.Request()

    def send_waypoint_request(self, waypoint_id):
        self.req.waypoint_id = waypoint_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().success

web_commander_client = None

def flask_thread():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    global web_commander_client
    rclpy.init(args=args)
    web_commander_client = WebCommanderClient()

    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(web_commander_client,))
    ros_spin_thread.daemon = True
    ros_spin_thread.start()

    flask_app_thread = threading.Thread(target=flask_thread)
    flask_app_thread.daemon = True
    flask_app_thread.start()

    # Keep the main thread alive, or join threads if you want to wait for them to finish
    # For a web server, you typically want it to run indefinitely
    try:
        while rclpy.ok():
            time.sleep(0.1) # Small sleep to prevent busy-waiting
    except KeyboardInterrupt:
        pass
    finally:
        web_commander_client.destroy_node()
        rclpy.shutdown()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_waypoint', methods=['POST'])
def send_waypoint():
    waypoint_id = request.json.get('waypoint_id')
    if waypoint_id is None:
        return jsonify({'success': False, 'message': 'waypoint_id not provided'}), 400
    
    try:
        waypoint_id = int(waypoint_id)
    except ValueError:
        return jsonify({'success': False, 'message': 'Invalid waypoint_id'}), 400

    if web_commander_client:
        success = web_commander_client.send_waypoint_request(waypoint_id)
        if success:
            return jsonify({'success': True, 'message': f'Successfully sent robot to waypoint {waypoint_id}'})
        else:
            return jsonify({'success': False, 'message': f'Failed to send robot to waypoint {waypoint_id}'}), 500
    else:
        return jsonify({'success': False, 'message': 'ROS 2 client not initialized'}), 500

if __name__ == '__main__':
    main()