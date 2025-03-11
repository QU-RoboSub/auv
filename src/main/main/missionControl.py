import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


class MissionControl(Node):
    """
    Mission Control Node for AUV
    Manages high-level decisions and task selection based on sensor inputs
    """

    def __init__(self):
        super().__init__('missionControlNode')
        
        #####################################################
        # SECTION: ROS PUBLISHERS AND SUBSCRIBERS
        #####################################################
        
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

        #####################################################
        # SECTION: STATE AND CONFIGURATION
        #####################################################
        
        # Storage for latest sensor data
        self.latest_sensors_data = []
        
        # Target class to filter (default: class 1)
        self.target_class = 1
        
        # Available tasks mapping (task_name: description)
        self.available_tasks = {
            "track_target": "Track and follow detected target",
            "hold_position": "Maintain current position and orientation",
            "depth_control": "Maintain specific depth while allowing other movements",
            "search_pattern": "Execute search pattern to find targets",
            "return_home": "Return to predefined home position"
        }
        
        # Default task settings
        self.current_task = "search_pattern"  # Start with search mode
        self.task_timeout = 10.0  # Seconds to continue track_target without new detections
        self.last_detection_time = self.get_clock().now()
        self.holding_timeout = 5.0  # Time to hold position after bottom camera detection

        # Task timer - periodically evaluate if tasks need to change
        self.task_timer = self.create_timer(1.0, self.task_evaluation_callback)
        
        self.get_logger().info("Mission Control initialized with target class: {}".format(self.target_class))
        self.get_logger().info("Available tasks: {}".format(", ".join(self.available_tasks.keys())))
        
        # Start the initial task
        self.send_task_command(self.current_task)

    #####################################################
    # SECTION: CAMERA INPUT PROCESSING
    #####################################################

    def listener_callback_front_yolo(self, msg):
        """
        Process front camera YOLO detections
        - Filters for target class
        - Switches to tracking task when target detected
        - Forwards filtered detections to Navigation
        """
        self.get_logger().info('Front YOLO data received')
        
        # Filter detections for the target class
        filtered_detections = []
        target_detected = False
        
        if len(msg.data) > 0:
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    
                    # Check if this is our target class
                    if int(class_id) == self.target_class:
                        target_detected = True
                        self.get_logger().info(
                            f'  TARGET DETECTED: Class {int(class_id)}, ' +
                            f'Confidence: {confidence:.2f}, ' + 
                            f'BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]'
                        )
                        
                        # Add to filtered detections
                        filtered_detections.extend([confidence, x1, y1, x2, y2, class_id])
                        
                        # Update last detection time
                        self.last_detection_time = self.get_clock().now()
                        
                        # Switch to tracking task if we're not already tracking
                        if self.current_task != "track_target":
                            self.current_task = "track_target"
                            self.send_task_command("track_target")
                    else:
                        self.get_logger().debug(f'  Ignoring detection: Class {int(class_id)} (not target class {self.target_class})')
        
        # If we found target class detections, forward them to Navigation
        if filtered_detections:
            forward_msg = Float32MultiArray()
            forward_msg.data = filtered_detections
            self.publisher_yolo_forward.publish(forward_msg)
            self.get_logger().info(f'Forwarded {len(filtered_detections) // 6} filtered detections to Navigation')
        else:
            # If no detections but we're tracking, check timeout
            if self.current_task == "track_target":
                current_time = self.get_clock().now()
                time_since_detection = (current_time - self.last_detection_time).nanoseconds / 1e9
                
                if time_since_detection > self.task_timeout:
                    self.get_logger().info(f'Target lost for {time_since_detection:.1f}s, switching to search')
                    self.current_task = "search_pattern"
                    self.send_task_command("search_pattern")

    def listener_callback_bottom_yolo(self, msg):
        """
        Process bottom camera YOLO detections
        - Switches to hold position when target detected
        """
        self.get_logger().info('Bottom YOLO data received')
        
        # Filter detections for the target class
        filtered_detections = []
        target_detected = False
        
        if len(msg.data) > 0:
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    
                    # Check if this is our target class
                    if int(class_id) == self.target_class:
                        target_detected = True
                        self.get_logger().info(
                            f'  BOTTOM TARGET DETECTED: Class {int(class_id)}, ' +
                            f'Confidence: {confidence:.2f}, ' + 
                            f'BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]'
                        )
                        
                        # When we see target on bottom camera, switch to hold position
                        if self.current_task in ["track_target", "search_pattern"]:
                            self.current_task = "hold_position"
                            self.send_task_command("hold_position")
                            self.last_detection_time = self.get_clock().now()  # Reset timer for holding
                    else:
                        self.get_logger().debug(f'  Ignoring detection: Class {int(class_id)} (not target class {self.target_class})')

    #####################################################
    # SECTION: OTHER SENSOR PROCESSING
    #####################################################

    def listener_callback_sensors(self, msg):
        """Process sensor data (orientation, depth)"""
        self.latest_sensors_data = msg.data
        
        if len(msg.data) > 0 and len(msg.data) >= 3:
            # Just store the data for mission decisions, logging at debug level
            roll, pitch, yaw = msg.data[0:3]
            self.get_logger().debug(f'Sensor data - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')
            
            # You could add logic here for depth-based task changes
            # For example, if at critical depth, force depth_control task

    #####################################################
    # SECTION: TASK MANAGEMENT
    #####################################################

    def send_task_command(self, task_name):
        """Send a task command to the navigation node"""
        if task_name in self.available_tasks:
            msg = String()
            msg.data = task_name
            self.publisher_task_command.publish(msg)
            self.get_logger().info(f'Sent task command: {task_name} - {self.available_tasks[task_name]}')
        else:
            self.get_logger().warn(f'Attempted to send unknown task: {task_name}')
    
    def task_evaluation_callback(self):
        """
        Periodically evaluate if tasks need to change
        - Publishes current task status
        - Handles timeouts for tasks
        """
        # Publish current target and task information
        target_msg = String()
        target_msg.data = f"class:{self.target_class},task:{self.current_task}"
        self.publisher_target.publish(target_msg)
        self.get_logger().debug(f'Current mission: class {self.target_class}, task: {self.current_task}')
        
        # Check if we need to change tasks based on timeouts
        current_time = self.get_clock().now()
        time_since_detection = (current_time - self.last_detection_time).nanoseconds / 1e9
        
        # If we're holding position after bottom camera detection, check timeout
        if self.current_task == "hold_position":
            if time_since_detection > self.holding_timeout:
                self.get_logger().info(f'Held position for {time_since_detection:.1f}s, switching to search')
                self.current_task = "search_pattern"
                self.send_task_command("search_pattern")
    
    #####################################################
    # SECTION: PAYLOAD MANAGEMENT
    #####################################################
    
    def timer_callback_payload_signal(self):
        """Send periodic signals to payload systems"""
        msg = String()
        msg.data = 'payload_command'  # Replace with actual payload command
        self.publisher_payload_signal.publish(msg)
        self.get_logger().debug('Publishing payload signal: "%s"' % msg.data)


#####################################################
# SECTION: MAIN ENTRY POINT
#####################################################

def main(args=None):
    rclpy.init(args=args)

    missionControlNode = MissionControl()

    rclpy.spin(missionControlNode)

    missionControlNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()