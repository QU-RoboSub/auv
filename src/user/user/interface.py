import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tkinter import Tk, Label
from PIL import Image as PILImage, ImageTk


class Interface(Node):

    def __init__(self):
        super().__init__('interfaceNode')

        # Initialize GUI components
        self.root = Tk()
        self.root.title("ROS Interface GUI")

        self.front_feed_label = Label(self.root)
        self.front_feed_label.grid(row=0, column=0, padx=10, pady=10)

        self.sensors_data_label = Label(self.root, text="Sensors Data: ")
        self.sensors_data_label.grid(row=1, column=0, padx=10, pady=10)

        self.target_data_label = Label(self.root, text="Target: ")
        self.target_data_label.grid(row=2, column=0, padx=10, pady=10)

        self.thrusters_data_label = Label(self.root, text="Thrusters Data: ")
        self.thrusters_data_label.grid(row=3, column=0, padx=10, pady=10)

        # Create Subscribers
        self.sensors_subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback_sensors,
            10)
        self.target_subscription = self.create_subscription(
            String,
            'target',
            self.listener_callback_target,
            10)
        self.thrusters_subscription = self.create_subscription(
            String,
            'input',
            self.listener_callback_thrusters,
            10)
        self.frontFeed_subscription = self.create_subscription(
            Image,
            'frontFeed',
            self.listener_callback_frontFeed,
            10)

        # Create bridge for image conversion
        self.bridge = CvBridge()

        # ROS logger
        self.get_logger().info('Interface Node started')

    def listener_callback_sensors(self, msg):
        # Use `after()` to schedule the GUI update on the main thread
        self.root.after(0, self.update_sensors_data, msg.data)

    def listener_callback_target(self, msg):
        # Use `after()` to schedule the GUI update on the main thread
        self.root.after(0, self.update_target_data, msg.data)

    def listener_callback_thrusters(self, msg):
        # Use `after()` to schedule the GUI update on the main thread
        self.root.after(0, self.update_thrusters_data, msg.data)

    def listener_callback_frontFeed(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to PIL Image to display in Tkinter
        pil_image = PILImage.fromarray(cv_image)
        pil_image = pil_image.resize((640, 480))  # Resize to fit the GUI

        # Convert the image for Tkinter display
        tk_image = ImageTk.PhotoImage(pil_image)

        # Use `after()` to update the image in the main thread
        self.root.after(0, self.update_front_feed, tk_image)

    def update_sensors_data(self, data):
        # Update the label with the sensors data
        self.sensors_data_label.config(text=f"Sensors Data: {data}")

    def update_target_data(self, data):
        # Update the label with the target data
        self.target_data_label.config(text=f"Target: {data}")

    def update_thrusters_data(self, data):
        # Update the label with the thrusters data
        self.thrusters_data_label.config(text=f"Thrusters Data: {data}")

    def update_front_feed(self, tk_image):
        # Update the label with the new image
        self.front_feed_label.config(image=tk_image)
        self.front_feed_label.image = tk_image  # Keep a reference to the image

    def run_gui(self):
        # Start the Tkinter GUI event loop in the main thread
        self.root.mainloop()

    def destroy_node(self):
        # Close the GUI window when shutting down
        self.root.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    interfaceNode = Interface()

    # Run ROS spin in the background in a separate thread
    def ros_spin():
        rclpy.spin(interfaceNode)

    # Start the ROS spinner in a separate thread so it doesn't block the GUI
    import threading
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Start the Tkinter GUI loop in the main thread
    interfaceNode.run_gui()

    interfaceNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
