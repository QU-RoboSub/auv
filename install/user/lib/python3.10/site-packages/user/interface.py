import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Interface(Node):

    def __init__(self):
        super().__init__('interfaceNode')

        # Create Orientation Subscriber
        self.sensors_subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback_sensors,
            10)
        self.sensors_subscription

        # Create User Target Subscriber
        self.target_subscription = self.create_subscription(
            String,
            'target',
            self.listener_callback_target,
            10)
        self.target_subscription

        # Create User Input Subscriber
        self.thrusters_subscription = self.create_subscription(
            String,
            'input',
            self.listener_callback_thrusters,
            10)
        self.thrusters_subscription

        # Create Front Feed Subscriber (for Image)
        self.frontFeed_subscription = self.create_subscription(
            Image,
            'frontFeed',
            self.listener_callback_frontFeed,
            10)
        self.frontFeed_subscription

        self.bridge = CvBridge()  # Initialize CvBridge to convert ROS Image to OpenCV image
        self.get_logger().info('Interface Node started')

    # Sensors Subscriber
    def listener_callback_sensors(self, msg):
        self.get_logger().info('Received sensors data: "%s"' % msg.data)

    # Target Subscriber
    def listener_callback_target(self, msg):
        self.get_logger().info('Received target data: "%s"' % msg.data)

    # Front Yolo Subscriber
    def listener_callback_frontFeed(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display the image using OpenCV
        cv2.imshow("Received Video Feed", cv_image)
        
        # Wait for key press (non-blocking)
        cv2.waitKey(1)

    # Thrusters Subscriber
    def listener_callback_thrusters(self, msg):
        self.get_logger().info('Received thrusters data: "%s"' % msg.data)

    def destroy_node(self):
        # Cleanup OpenCV windows when shutting down
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    interfaceNode = Interface()

    rclpy.spin(interfaceNode)

    interfaceNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
