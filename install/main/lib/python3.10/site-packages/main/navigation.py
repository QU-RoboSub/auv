import rclpy
from rclpy.node import Node
import numpy as np
import math

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
        
        # Create Orientation Publisher (will send deviations to thrusters)
        self.publisher_orientation = self.create_publisher(
            Float32MultiArray, 
            'orientation', 
            10)
        timer_period_orientation = 0.1  # 10Hz update rate
        self.timer_orientation = self.create_timer(timer_period_orientation, self.timer_callback_orientation)

        # Initialize navigation state
        self.current_task = "none"
        self.target_detections = []
        
        # Tracking parameters
        self.angle_threshold = 5.0  # Degrees - when to consider target centered
        self.distance_threshold = 300.0  # mm - when to consider target at right distance
        self.last_valid_angle = 0.0
        self.last_valid_distance = 0.0
        
        # Camera parameters (match with depth camera settings)
        self.camera_fov_horizontal = 73.0  # FOV in degrees
        self.camera_preview_width = 640
        self.camera_preview_height = 480
        
        # Sensor data storage
        self.sensor_data = {
            'depth': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # Control parameters
        self.target_depth = 0.5  # meters
        self.target_roll = 0.0   # degrees
        self.target_pitch = 0.0  # degrees
        self.target_yaw = None   # Will be set based on current yaw when needed
        
        # Deviation parameters sent to thrusters
        self.roll_deviation = 0.0
        self.pitch_deviation = 0.0
        self.yaw_deviation = 0.0
        self.surge_value = 0.0   # -2 to 2, affects vertical movement
        self.sway_value = 0.0    # -2 to 2, affects lateral movement
        
        # Available task handlers with better tracking capabilities
        self.task_handlers = {
            "track_target": self.task_track_target,
            "hold_position": self.task_hold_position,
            "depth_control": self.task_depth_control,
            "search_pattern": self.task_search_pattern,
            "return_home": self.task_return_home
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
            # Process YOLO detections and calculate tracking parameters
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    confidence, x1, y1, x2, y2, class_id = msg.data[i:i+6]
                    
                    # Calculate detection parameters
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Calculate angle to target (similar to IMU's tracking algorithm)
                    offset_x = x_center - (self.camera_preview_width / 2)
                    horizontal_angle = float(offset_x * (self.camera_fov_horizontal / self.camera_preview_width))
                    
                    # Estimate distance based on bounding box size
                    # This is a simple approximation - real distance would be from depth camera
                    z_distance = 5000.0 / (width + 0.1)  # Avoid division by zero
                    
                    self.get_logger().info(
                        f'  Detection {i//6 + 1}: Class {int(class_id)}, ' +
                        f'Confidence: {confidence:.2f}, ' +
                        f'Angle: {horizontal_angle:.2f}°, ' +
                        f'Est. Distance: {z_distance:.2f}mm'
                    )
                    
                    # Store detection with tracking data
                    self.target_detections.append({
                        'confidence': confidence,
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'center_x': x_center, 'center_y': y_center,
                        'width': width, 'height': height,
                        'class_id': int(class_id),
                        'angle': horizontal_angle,
                        'distance': z_distance
                    })
            
            # Update tracking parameters based on detections
            self.update_tracking_parameters()
            
            # Execute current task with new detections
            self.execute_current_task()
        else:
            self.get_logger().info('YOLO: No target detections')
            # Even without detections, maintain the last valid values for a short time
            # This helps with brief tracking interruptions
            self.execute_current_task()

    # Update tracking parameters based on latest detections
    def update_tracking_parameters(self):
        if not self.target_detections:
            # No detections - values will decay over time
            # For now we keep the last values
            return
            
        # Use the last detection instead of the best detection
        last_detection = self.target_detections[-1] if self.target_detections else None
        
        if last_detection:
            # Update angle and distance with some smoothing
            current_angle = last_detection['angle']
            current_distance = last_detection['distance']
        
        # Apply thresholds for small angles and distances (like in IMU code)
        if abs(current_angle) <= self.angle_threshold:
            # Person is roughly centered
            self.last_valid_angle = 0.0
        else:
            # Update for tracking
            self.last_valid_angle = current_angle
            
        if abs(current_distance - self.distance_threshold) <= 300:
            # Person is at roughly the right distance
            self.last_valid_distance = 0.0
        else:
            # Update for tracking
            self.last_valid_distance = current_distance - self.distance_threshold

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
                
                # Add depth if available (first parameter)
                if len(msg.data) >= 4:
                    self.sensor_data['depth'] = msg.data[0]
                    roll, pitch, yaw = msg.data[1:4]
                    self.sensor_data['roll'] = roll
                    self.sensor_data['pitch'] = pitch
                    self.sensor_data['yaw'] = yaw
                
                self.get_logger().debug(
                    f'Orientation: Roll {self.sensor_data["roll"]:.2f}°, ' +
                    f'Pitch {self.sensor_data["pitch"]:.2f}°, ' +
                    f'Yaw {self.sensor_data["yaw"]:.2f}°'
                )
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

    # Calculate surge value based on distance to target
    def calculate_surge(self, distance):
        """
        Convert distance deviation to a surge value between -2 and 2
        Positive surge: move forward
        Negative surge: move backward
        """
        # If distance is near threshold, no forward/backward movement needed
        if abs(distance) < 50:
            return 0.0
            
        # Map distance to surge value in range [-2, 2]
        # This is a simple linear mapping - can be adjusted based on testing
        max_distance = 2000  # Maximum distance deviation we care about
        surge = np.clip(distance / max_distance * 2.0, -2.0, 2.0)
        
        return float(surge)

    # Update and publish orientation deviations to thrusters
    def send_orientation_deviations(self, roll_dev=0.0, pitch_dev=0.0, yaw_dev=0, surge=None, sway=0.0):
        """
        Send deviation commands to the thrusters via the orientation topic
        All parameters are optional and default to current values if not specified
        """
        if surge is None:
            # Calculate surge from distance if not specified
            surge = self.calculate_surge(self.last_valid_distance)
            
        # Update stored deviation values
        self.roll_deviation = float(roll_dev)
        self.pitch_deviation = float(pitch_dev)
        self.yaw_deviation = float(self.last_valid_angle)
        self.surge_value = float(np.clip(surge, -2.0, 2.0))
        self.sway_value = float(np.clip(sway, -2.0, 2.0))
        
        # Deviations will be applied in timer_callback_orientation
        
        self.get_logger().debug(
            f'Updated deviations - Roll: {self.roll_deviation:.2f}, ' +
            f'Pitch: {self.pitch_deviation:.2f}, ' +
            f'Yaw: {self.yaw_deviation:.2f}, ' +
            f'Surge: {self.surge_value:.2f}, ' +
            f'Sway: {self.sway_value:.2f}'
        )

    # Task handlers
    def task_track_target(self):
        """Track a detected target using vision-based control"""
        if self.target_detections:
            # We have detections - actively track the target
            self.get_logger().info(f'TASK TRACK TARGET: Tracking object with {len(self.target_detections)} detections')
            
            # Use the tracking parameters calculated earlier
            yaw_deviation = self.last_valid_angle
            
            # Calculate surge value based on distance
            distance_deviation = self.last_valid_distance
            
            # Set deviations - for tracking we primarily use yaw and surge
            self.send_orientation_deviations(
                roll_dev=0.0,       # No roll during tracking
                pitch_dev=0.0,      # No pitch during tracking
                yaw_dev=yaw_deviation,  # Yaw to track target horizontally
                surge=self.calculate_surge(distance_deviation),  # Forward/backward movement based on distance
                sway=0.0            # No lateral movement during tracking
            )
            
            # Log tracking information
            if abs(yaw_deviation) > self.angle_threshold:
                self.get_logger().info(f'  Adjusting yaw by {yaw_deviation:.2f}° to track target')
            else:
                self.get_logger().info('  Target centered horizontally')
                
            if abs(distance_deviation) > 50:
                direction = "forward" if self.surge_value > 0 else "backward" if self.surge_value < 0 else "stay level"
                self.get_logger().info(f'  Distance adjustment: {direction} (surge: {self.surge_value:.2f})')
            else:
                self.get_logger().info('  Target at optimal distance')
            
        else:
            # No detections - gradually reduce deviations to slow down when target is lost
            self.get_logger().info('TASK TRACK TARGET: No current detections, gradually stopping')
            
            # Decay the deviation values
            self.roll_deviation = 0
            self.pitch_deviation = 0
            self.yaw_deviation = 0
            self.surge_value = 0
            self.sway_value = 0
                        
            # Send the decayed deviations
            self.send_orientation_deviations(
                roll_dev=self.roll_deviation,
                pitch_dev=self.pitch_deviation,
                yaw_dev=self.yaw_deviation,
                surge=self.surge_value,
                sway=self.sway_value
            )

    def task_hold_position(self):
        """Maintain current position and orientation"""
        self.get_logger().info('TASK HOLD POSITION: Maintaining current position')
        
        # Send zero deviations to hold position
        self.send_orientation_deviations(
            roll_dev=0.0,
            pitch_dev=0.0,
            yaw_dev=0.0,
            surge=0.0,
            sway=0.0
        )

    def task_depth_control(self):
        """Maintain a specific depth while allowing other movements"""
        self.get_logger().info(f'TASK DEPTH CONTROL: Maintaining depth at {self.target_depth}m')
        
        # Get current depth
        current_depth = self.sensor_data.get('depth', 0.0)
        
        # Calculate depth deviation
        depth_error = self.target_depth - current_depth
        
        # Convert to surge value (-2 to 2)
        surge_value = np.clip(depth_error * 4.0, -2.0, 2.0)  # Scale factor of 4 for sensitivity
        
        self.get_logger().info(f'  Current depth: {current_depth:.2f}m, Target: {self.target_depth:.2f}m, Surge: {surge_value:.2f}')
        
        # Send deviations - only override surge for depth control
        self.send_orientation_deviations(surge=surge_value)

    def task_search_pattern(self):
        """Perform a search pattern to find targets"""
        self.get_logger().info('TASK SEARCH PATTERN: Executing search pattern')
        
        # Calculate a simple rotating search pattern
        current_time = self.get_clock().now().to_msg().sec % 20  # 20-second cycle
        
        # Calculate yaw deviation for search pattern
        if current_time < 10:
            # Rotate right
            yaw_deviation = 10.0
            self.get_logger().info('  Searching right')
        else:
            # Rotate left
            yaw_deviation = -10.0
            self.get_logger().info('  Searching left')
        
        # Send deviations for search pattern
        self.send_orientation_deviations(
            roll_dev=0.0,
            pitch_dev=0.0,
            yaw_dev=yaw_deviation,
            surge=0.0,  # No forward/backward movement during search
            sway=0.0    # No lateral movement during search
        )

    def task_return_home(self):
        """Return to a predefined home position"""
        self.get_logger().info('TASK RETURN HOME: Returning to home position')
        
        # Get current depth
        current_depth = self.sensor_data.get('depth', 0.0)
        
        # Target depth near surface
        home_depth = 0.3
        
        # Calculate depth deviation
        depth_error = home_depth - current_depth
        
        # Convert to surge value (-2 to 2)
        surge_value = np.clip(depth_error * 4.0, -2.0, 2.0)
        
        # For return home, we might want to reset orientation to neutral
        self.send_orientation_deviations(
            roll_dev=0.0,        # Level roll
            pitch_dev=0.0,       # Level pitch 
            yaw_dev=0.0,         # No yaw adjustment
            surge=surge_value,   # Move to home depth
            sway=0.0             # No lateral movement
        )

    # Orientation Publisher - sends deviations to thrusters
    def timer_callback_orientation(self):
        # Create message with deviations in format expected by thrusters
        msg = Float32MultiArray()
        msg.data = [
            float(self.roll_deviation),   # Roll deviation
            float(self.pitch_deviation),  # Pitch deviation
            float(self.yaw_deviation),    # Yaw deviation
            float(self.surge_value),      # Surge value (-2 to 2)
            float(self.sway_value)        # Sway value (-2 to 2)
        ]
        
        # Verify all values are valid floats
        if all(not math.isnan(x) and not math.isinf(x) for x in msg.data):
            self.publisher_orientation.publish(msg)
            
            # Only log at debug level to avoid flooding logs
            self.get_logger().debug(
                f'Published deviations: Roll {self.roll_deviation:.2f}, ' +
                f'Pitch {self.pitch_deviation:.2f}, ' +
                f'Yaw {self.yaw_deviation:.2f}, ' + 
                f'Surge {self.surge_value:.2f}, ' +
                f'Sway {self.sway_value:.2f}'
            )
        else:
            self.get_logger().warn("Invalid orientation values detected, skipping publish")


def main(args=None):
    rclpy.init(args=args)

    navigationNode = Navigation()

    rclpy.spin(navigationNode)

    navigationNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
