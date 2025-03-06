import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


class MissionControl(Node):

    def __init__(self):
        super().__init__('missionControlNode')
        # Create Front Yolo Subscriber
        self.front_yolo_subscription = self.create_subscription(
            Float32MultiArray,
            'frontYolo',
            self.listener_callback_front_yolo,
            10)
        self.front_yolo_subscription  

        # Create Bottom Yolo Subscriber
        self.bottom_yolo_subscription = self.create_subscription(
            Float32MultiArray,
            'bottomYolo',
            self.listener_callback_bottom_yolo,
            10)
        self.bottom_yolo_subscription

        # Subscribe to Sensors
        self.sensors_subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback_sensors,
            10)
        self.sensors_subscription

        # Create YOLO Forward Publisher (to Navigation)
        self.publisher_yolo_forward = self.create_publisher(
            Float32MultiArray, 
            'yolo_forward', 
            10)

        # Create Target Publisher (for navigation tasks)
        self.publisher_target = self.create_publisher(String, 'target', 10)
        
        # Create Task Command Publisher (to send navigation commands)
        self.publisher_task_command = self.create_publisher(String, 'task_command', 10)

        # Create Payload Signal Publisher
        self.publisher_payload_signal = self.create_publisher(String, 'payloadSignal', 10)
        timer_period_payload_signal = 1  # seconds
        self.payload_signal_timer = self.create_timer(timer_period_payload_signal, self.timer_callback_payload_signal)

        # Storage for latest sensor data
        self.latest_sensors_data = []
        
        # Target class to filter (default: class 1)
        self.target_class = 1
        
        # Available tasks mapping (task_name: description)
        self.available_tasks = {
            "forward": "Move forward toward detected object",
            "backward": "Move backward away from detected object",
            "stop": "Stop movement",
            "search": "Search for target objects"
        }
        
        # Current active task
        self.current_task = "search"
        
        # Task timer - periodically evaluate if tasks need to change
        self.task_timer = self.create_timer(1.0, self.task_evaluation_callback)
        
        self.get_logger().info("Mission Control initialized with target class: {}".format(self.target_class))
        self.get_logger().info("Available tasks: {}".format(", ".join(self.available_tasks.keys())))

    # Front Yolo Subscriber
    def listener_callback_front_yolo(self, msg):
        self.get_logger().info('Front YOLO data received')
        
        # Filter detections for the target class
        filtered_detections = []
        
        if len(msg.data) > 0:
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    
                    # Check if this is our target class
                    if int(class_id) == self.target_class:
                        self.get_logger().info(f'  TARGET DETECTED: Class {int(class_id)}, ' +
                                         f'Confidence: {confidence:.2f}, ' + 
                                         f'BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
                        
                        # Add to filtered detections
                        filtered_detections.extend([confidence, x1, y1, x2, y2, class_id])
                        
                        # If we detect our target class, switch to "forward" task
                        if self.current_task != "forward":
                            self.current_task = "forward"
                            self.send_task_command("forward")
                    else:
                        self.get_logger().debug(f'  Ignoring detection: Class {int(class_id)} (not target class {self.target_class})')
        
        # If we found target class detections, forward them to Navigation
        if filtered_detections:
            forward_msg = Float32MultiArray()
            forward_msg.data = filtered_detections
            self.publisher_yolo_forward.publish(forward_msg)
            self.get_logger().info(f'Forwarded {len(filtered_detections) // 6} filtered detections to Navigation')
        elif self.current_task == "forward":
            # If we were tracking an object but lost it, switch back to search mode
            self.current_task = "search"
            self.send_task_command("search")

    # Bottom Yolo Subscriber
    def listener_callback_bottom_yolo(self, msg):
        self.get_logger().info('Bottom YOLO data received')
        
        # Filter detections for the target class
        filtered_detections = []
        
        if len(msg.data) > 0:
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    
                    # Check if this is our target class
                    if int(class_id) == self.target_class:
                        self.get_logger().info(f'  TARGET DETECTED: Class {int(class_id)}, ' +
                                         f'Confidence: {confidence:.2f}, ' + 
                                         f'BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
                        
                        # Bottom camera detections could trigger different tasks
                        # For now, we'll just log them
                    else:
                        self.get_logger().debug(f'  Ignoring detection: Class {int(class_id)} (not target class {self.target_class})')

    # Sensors Subscriber
    def listener_callback_sensors(self, msg):
        self.latest_sensors_data = msg.data
        
        if len(msg.data) > 0 and len(msg.data) >= 3:
            roll, pitch, yaw = msg.data[0:3]
            self.get_logger().debug(f'Sensor data - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

    # Send a task command to the navigation node
    def send_task_command(self, task_name):
        if task_name in self.available_tasks:
            msg = String()
            msg.data = task_name
            self.publisher_task_command.publish(msg)
            self.get_logger().info(f'Sent task command: {task_name} - {self.available_tasks[task_name]}')
        else:
            self.get_logger().warn(f'Attempted to send unknown task: {task_name}')
    
    # Periodically evaluate if tasks need to change based on state
    def task_evaluation_callback(self):
        # This method can be expanded to have more complex task switching logic
        # For now, it just publishes the current target information
        target_msg = String()
        target_msg.data = f"class:{self.target_class},task:{self.current_task}"
        self.publisher_target.publish(target_msg)
        self.get_logger().debug(f'Current target: class {self.target_class}, task: {self.current_task}')
    
    # Payload Signal Publisher
    def timer_callback_payload_signal(self):
        msg = String()
        msg.data = 'payload_command'  # Replace with actual payload command
        self.publisher_payload_signal.publish(msg)
        self.get_logger().debug('Publishing payload signal: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    missionControlNode = MissionControl()

    rclpy.spin(missionControlNode)

    missionControlNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()