import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


class Navigation(Node):

    def __init__(self):
        super().__init__('navigationNode')
        # Subscribe to Target from Mission Control
        self.target_subscription = self.create_subscription(
            String,
            'target',
            self.listener_callback_target,
            10)
        self.target_subscription  # prevent unused variable warning
        
        # Subscribe to Forward YOLO data from Mission Control
        self.yolo_subscription = self.create_subscription(
            Float32MultiArray,
            'yolo_forward',
            self.listener_callback_yolo,
            10)
        self.yolo_subscription  # prevent unused variable warning
        
        # Subscribe directly to Sensors
        self.sensors_subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback_sensors,
            10)
        self.sensors_subscription  # prevent unused variable warning
        
        # Subscribe to Task Commands from Mission Control
        self.task_cmd_subscription = self.create_subscription(
            String,
            'task_command',
            self.listener_callback_task_command,
            10)
        self.task_cmd_subscription  # prevent unused variable warning
        
        # Create Orientation Publisher
        self.publisher_orientation = self.create_publisher(String, 'orientation', 10)
        timer_period_orientation = 1  # seconds
        self.timer_orientation = self.create_timer(timer_period_orientation, self.timer_callback_orientation)

        # Initialize navigation state
        self.current_task = "none"
        self.target_detections = []
        self.sensor_data = {}
        
        # Available task handlers
        self.task_handlers = {
            "forward": self.task_forward,
            "backward": self.task_backward,
            "stop": self.task_stop,
            "search": self.task_search
        }
        
        self.get_logger().info("Navigation node initialized with available tasks: {}".format(
            ", ".join(self.task_handlers.keys())))

    # Target Subscriber   
    def listener_callback_target(self, msg):
        self.get_logger().info('Target info received: "%s"' % msg.data)
        # Parse target information (format: class:X,task:Y)
        try:
            parts = msg.data.split(',')
            if len(parts) >= 2:
                class_part = parts[0].split(':')
                task_part = parts[1].split(':')
                
                if len(class_part) == 2 and len(task_part) == 2:
                    target_class = int(class_part[1])
                    target_task = task_part[1]
                    
                    self.get_logger().info(f'Current mission: Track class {target_class}, task: {target_task}')
        except Exception as e:
            self.get_logger().warn(f'Error parsing target info: {str(e)}')

    # YOLO Forward Subscriber
    def listener_callback_yolo(self, msg):
        # Store the latest detections
        self.target_detections = []
        
        if len(msg.data) > 0:
            self.get_logger().info('YOLO target detections received:')
            # Assuming YOLO data format is [confidence, x1, y1, x2, y2, class_id, ...]
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    self.get_logger().info(f'  Detection {i//6 + 1}: Class {int(class_id)}, Confidence: {confidence:.2f}, BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
                    
                    # Store detection for task execution
                    self.target_detections.append({
                        'confidence': confidence,
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'class_id': int(class_id)
                    })
            
            # Execute current task with new detections
            self.execute_current_task()
        else:
            self.get_logger().info('YOLO: No target detections')

    # Sensors Subscriber
    def listener_callback_sensors(self, msg):
        if len(msg.data) > 0:
            # Store sensor data for tasks
            if len(msg.data) >= 3:
                roll, pitch, yaw = msg.data[0:3]
                self.sensor_data = {
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw
                }
                
                self.get_logger().debug(f'Orientation: Roll {roll:.2f}°, Pitch {pitch:.2f}°, Yaw {yaw:.2f}°')
                
                # If there's additional sensor data
                if len(msg.data) > 3:
                    self.get_logger().debug(f'Additional sensor data: {len(msg.data)-3} values')
        else:
            self.get_logger().debug('Sensors: No data received')

    # Task Command Subscriber
    def listener_callback_task_command(self, msg):
        task_name = msg.data
        self.get_logger().info(f'Received task command: {task_name}')
        
        if task_name in self.task_handlers:
            self.current_task = task_name
            self.get_logger().info(f'Switching to task: {task_name}')
            # Execute the task immediately
            self.execute_current_task()
        else:
            self.get_logger().warn(f'Unknown task command: {task_name}')

    # Execute the current task
    def execute_current_task(self):
        if self.current_task in self.task_handlers:
            handler = self.task_handlers[self.current_task]
            handler()
        else:
            self.get_logger().warn(f'No handler found for task: {self.current_task}')

    # Task handlers
    def task_forward(self):
        """Move forward toward detected object"""
        if self.target_detections:
            # For this example, we'll just calculate the center point of the first detection
            detection = self.target_detections[0]
            center_x = (detection['x1'] + detection['x2']) / 2
            center_y = (detection['y1'] + detection['y2']) / 2
            box_width = detection['x2'] - detection['x1']
            box_height = detection['y2'] - detection['y1']
            
            # Calculate if we need to move left/right and up/down
            frame_center_x = 640 / 2  # Assuming frame width of 640
            frame_center_y = 480 / 2  # Assuming frame height of 480
            
            x_error = center_x - frame_center_x
            y_error = center_y - frame_center_y
            
            # Simple text-based navigation output for demonstration
            self.get_logger().info(f'TASK FORWARD: Tracking object {detection["class_id"]}')
            self.get_logger().info(f'  Object center: ({center_x:.1f}, {center_y:.1f}), Size: {box_width:.1f} x {box_height:.1f}')
            
            if abs(x_error) > 50:
                direction = "right" if x_error > 0 else "left"
                self.get_logger().info(f'  Need to move {direction} to center object (error: {x_error:.1f}px)')
            else:
                self.get_logger().info(f'  Object centered horizontally (error: {x_error:.1f}px)')
                
            if abs(y_error) > 50:
                direction = "down" if y_error > 0 else "up"
                self.get_logger().info(f'  Need to move {direction} to center object (error: {y_error:.1f}px)')
            else:
                self.get_logger().info(f'  Object centered vertically (error: {y_error:.1f}px)')
                
            # Calculate distance (simple approximation based on bounding box size)
            # Larger bounding box = closer object
            if box_width < 100:
                self.get_logger().info('  Need to move forward to approach object (object is far)')
            elif box_width > 300:
                self.get_logger().info('  Need to stop or slow down (object is close)')
            else:
                self.get_logger().info('  Maintain current distance to object')
        else:
            self.get_logger().info('TASK FORWARD: No target detections, unable to execute forward task')

    def task_backward(self):
        """Move backward away from detected object"""
        self.get_logger().info('TASK BACKWARD: Moving away from object')
        # Implementation would go here

    def task_stop(self):
        """Stop all movement"""
        self.get_logger().info('TASK STOP: Stopping all movement')
        # Implementation would go here

    def task_search(self):
        """Search for target objects"""
        self.get_logger().info('TASK SEARCH: Searching for target objects')
        # Implementation would go here - could involve rotating in place, scanning area, etc.

    # Orientation Publisher
    def timer_callback_orientation(self):
        msg = String()
        msg.data = ''
        self.publisher_orientation.publish(msg)
        self.get_logger().debug('Publishing orientation: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    navigationNode = Navigation()

    rclpy.spin(navigationNode)

    navigationNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
