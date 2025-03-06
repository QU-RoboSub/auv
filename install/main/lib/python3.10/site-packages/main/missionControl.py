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

        # Create Target Publisher
        self.publisher_target = self.create_publisher(String, 'target', 10)
        timer_period_target = 1  # seconds
        self.target_timer = self.create_timer(timer_period_target, self.timer_callback_target)

        # Create Payload Signal Publisher
        self.publisher_payload_signal = self.create_publisher(String, 'payloadSignal', 10)
        timer_period_payload_signal = 1  # seconds
        self.payload_signal_timer = self.create_timer(timer_period_payload_signal, self.timer_callback_payload_signal)

        # Storage for latest sensor data
        self.latest_sensors_data = []

    # Front Yolo Subscriber
    def listener_callback_front_yolo(self, msg):
        self.get_logger().info('Front YOLO data received')
        
        # Log YOLO data
        if len(msg.data) > 0:
            # Assuming YOLO data format is [x1, y1, x2, y2, confidence, class_id, ...]
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    x1, y1, x2, y2, conf, class_id = msg.data[i:i+6]
                    self.get_logger().info(f'  Detection {i//6 + 1}: Class {int(class_id)}, Confidence: {conf:.2f}, BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
        
        # Forward YOLO data to Navigation
        forward_msg = Float32MultiArray()
        forward_msg.data = msg.data
        self.publisher_yolo_forward.publish(forward_msg)
        self.get_logger().info('Forwarded YOLO data to Navigation')

    # Bottom Yolo Subscriber
    def listener_callback_bottom_yolo(self, msg):
        self.get_logger().info('Bottom YOLO data received')
        
        # Log YOLO data
        if len(msg.data) > 0:
            # Assuming YOLO data format is [x1, y1, x2, y2, confidence, class_id, ...]
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    x1, y1, x2, y2, conf, class_id = msg.data[i:i+6]
                    self.get_logger().info(f'  Detection {i//6 + 1}: Class {int(class_id)}, Confidence: {conf:.2f}, BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')

    # Sensors Subscriber
    def listener_callback_sensors(self, msg):
        self.latest_sensors_data = msg.data
        
        if len(msg.data) > 0:
            self.get_logger().info('Sensor data received:')
            # Assuming sensor data format is [depth, roll, pitch, yaw, ...]
            if len(msg.data) >= 4:
                depth, roll, pitch, yaw = msg.data[0:4]
                self.get_logger().info(f'  Depth: {depth:.2f}m, Orientation: Roll {roll:.2f}°, Pitch {pitch:.2f}°, Yaw {yaw:.2f}°')
                
                # Print any additional sensor data
                if len(msg.data) > 4:
                    self.get_logger().info(f'  Additional sensor data: {msg.data[4:]}')
        else:
            self.get_logger().info('Sensors: No data received')

    # Target Publisher
    def timer_callback_target(self):
        msg = String()
        msg.data = 'current_target'  # Replace with actual target calculation based on sensor data
        self.publisher_target.publish(msg)
        self.get_logger().info('Publishing target: "%s"' % msg.data)
    
    # Payload Signal Publisher
    def timer_callback_payload_signal(self):
        msg = String()
        msg.data = 'payload_command'  # Replace with actual payload command
        self.publisher_payload_signal.publish(msg)
        self.get_logger().info('Publishing payload signal: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    missionControlNode = MissionControl()

    rclpy.spin(missionControlNode)

    missionControlNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()