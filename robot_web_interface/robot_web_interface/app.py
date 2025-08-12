import rclpy
from rclpy.node import Node
from robot_commander_interfaces.srv import SendWaypoint
from flask import Flask, render_template, request, jsonify
import threading
import time

app = Flask(__name__)

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