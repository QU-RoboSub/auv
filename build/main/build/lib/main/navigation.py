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
        
        # Create Orientation Publisher
        self.publisher_orientation = self.create_publisher(String, 'orientation', 10)
        timer_period_orientation = 1  # seconds
        self.timer_orientation = self.create_timer(timer_period_orientation, self.timer_callback_orientation)

    # Target Subscriber   
    def listener_callback_target(self, msg):
        self.get_logger().info('Target received: "%s"' % msg.data)

    # YOLO Forward Subscriber
    def listener_callback_yolo(self, msg):
        if len(msg.data) > 0:
            self.get_logger().info('YOLO detections received:')
            # Assuming YOLO data format is [x1, y1, x2, y2, confidence, class_id, ...]
            for i in range(0, len(msg.data), 6):
                if i + 5 < len(msg.data):
                    x1, y1, x2, y2, conf, class_id = msg.data[i:i+6]
                    self.get_logger().info(f'  Detection {i//6 + 1}: Class {int(class_id)}, Confidence: {conf:.2f}, BBox: [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]')
        else:
            self.get_logger().info('YOLO: No detections')

    # Sensors Subscriber
    def listener_callback_sensors(self, msg):
        if len(msg.data) > 0:
            self.get_logger().info('Sensor data received:')
            # Process based on the data length
            if len(msg.data) == 3:
                # Standard case: Just roll, pitch, yaw
                roll, pitch, yaw = msg.data
                self.get_logger().info(f'  Orientation: Roll {roll:.2f}°, Pitch {pitch:.2f}°, Yaw {yaw:.2f}°')
            elif len(msg.data) >= 4:
                # If depth is included or other data
                depth = msg.data[0]
                roll, pitch, yaw = msg.data[1:4]
                self.get_logger().info(f'  Depth: {depth:.2f}m, Orientation: Roll {roll:.2f}°, Pitch {pitch:.2f}°, Yaw {yaw:.2f}°')
                
                # If there's additional data, log it but avoid printing excessive arrays
                if len(msg.data) > 4:
                    additional_count = len(msg.data) - 4
                    self.get_logger().debug(f'  Additional sensor data: {additional_count} values')
                    # Only print the first few values to avoid cluttering the logs
                    if additional_count <= 10:
                        self.get_logger().debug(f'  Values: {msg.data[4:]}')
                    else:
                        self.get_logger().debug(f'  First 5 values: {msg.data[4:9]}...')
        else:
            self.get_logger().info('Sensors: No data received')

    # Orientation Publisher
    def timer_callback_orientation(self):
        msg = String()
        msg.data = ''
        self.publisher_orientation.publish(msg)
        self.get_logger().info('Publishing orientation: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    navigationNode = Navigation()

    rclpy.spin(navigationNode)

    navigationNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
