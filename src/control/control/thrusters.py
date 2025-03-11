import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class Thrusters(Node):

    def __init__(self):
        super().__init__('thrustersNode')


    #Create Orientation Subscriber
        self.orientation_subscription = self.create_subscription(
        Float32MultiArray,
        'orientation',
        self.listener_callback_orientation,
        10)
        self.orientation_subscription  

     #Create User Input Subscriber
        self.input_subscription = self.create_subscription(
        String,
        'input',
        self.listener_callback_input,
        10)
        self.input_subscription  

    #Sensors Subscriber
    def listener_callback_sensors(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    #Orientation Subscriber
    def listener_callback_orientation(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    thrustersNode = Thrusters()

    rclpy.spin(thrustersNode)

    thrustersNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()