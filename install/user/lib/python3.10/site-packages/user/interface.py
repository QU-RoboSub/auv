import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tkinter import Tk, Frame, Label, Button
from tkinter.ttk import Style
from PIL import Image as PILImage, ImageTk, ImageDraw

class Interface(Node):

    def __init__(self):
        super().__init__('interfaceNode')

        # Initialize GUI components
        self.root = Tk()
        self.root.title("ROS Interface GUI")
        self.root.geometry("1200x800")  # Set a larger window size
        self.root.configure(bg="#2E3440")  # Dark background color

        # Custom style for modern look
        self.style = Style()
        self.style.configure("TFrame", background="#3B4252")  # Darker frame background
        self.style.configure("TButton", background="#81A1C1", foreground="#ECEFF4", font=("Helvetica", 10), borderwidth=0)
        self.style.map("TButton", background=[("active", "#5E81AC")])  # Button hover effect

        # Create a frame for the live feed and buttons
        self.main_frame = Frame(self.root, bg="#2E3440")
        self.main_frame.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")  # Use grid instead of pack
        self.root.grid_rowconfigure(0, weight=1)  # Make the main_frame row resizable
        self.root.grid_columnconfigure(0, weight=1)  # Make the main_frame column resizable

        # Create a frame for the live feed
        self.feed_frame = Frame(self.main_frame, bg="#3B4252", bd=0, highlightthickness=0)
        self.feed_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")  # Allow feed_frame to expand

        # Allow the grid to expand in both directions
        self.main_frame.grid_rowconfigure(0, weight=1)  # Make the row with the feed_frame resizable
        self.main_frame.grid_columnconfigure(0, weight=1)  # Make the column with the feed_frame resizable

        # Create a label for the front feed
        self.front_feed_label = Label(self.feed_frame, bg="#3B4252")
        self.front_feed_label.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")  # Allow the label to fill the frame

        # Create a frame for the DOF control buttons (right side of the live feed)
        self.control_frame = Frame(self.main_frame, bg="#3B4252", bd=0, highlightthickness=0)
        self.control_frame.grid(row=0, column=1, padx=10, pady=10)

        # Create buttons for each degree of freedom with modern styling
        self.front_button = self.create_rounded_button("▲", self.move_front)
        self.front_button.grid(row=0, column=1, padx=5, pady=5)

        self.back_button = self.create_rounded_button("▼", self.move_back)
        self.back_button.grid(row=2, column=1, padx=5, pady=5)

        self.left_button = self.create_rounded_button("◄", self.move_left)
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.right_button = self.create_rounded_button("►", self.move_right)
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        self.depth_up_button = self.create_rounded_button("Depth ▲", self.depth_up)
        self.depth_up_button.grid(row=3, column=1, padx=5, pady=5)

        self.depth_down_button = self.create_rounded_button("Depth ▼", self.depth_down)
        self.depth_down_button.grid(row=4, column=1, padx=5, pady=5)

        self.roll_left_button = self.create_rounded_button("Roll ◄", self.roll_left)
        self.roll_left_button.grid(row=5, column=0, padx=5, pady=5)

        self.roll_right_button = self.create_rounded_button("Roll ►", self.roll_right)
        self.roll_right_button.grid(row=5, column=2, padx=5, pady=5)

        self.pitch_up_button = self.create_rounded_button("Pitch ▲", self.pitch_up)
        self.pitch_up_button.grid(row=6, column=1, padx=5, pady=5)

        self.pitch_down_button = self.create_rounded_button("Pitch ▼", self.pitch_down)
        self.pitch_down_button.grid(row=7, column=1, padx=5, pady=5)

        self.yaw_left_button = self.create_rounded_button("Yaw ◄", self.yaw_left)
        self.yaw_left_button.grid(row=8, column=0, padx=5, pady=5)

        self.yaw_right_button = self.create_rounded_button("Yaw ►", self.yaw_right)
        self.yaw_right_button.grid(row=8, column=2, padx=5, pady=5)

        # Create a frame for the bottom labels (to hold them side by side)
        self.bottom_frame = Frame(self.root, bg="#3B4252", bd=0, highlightthickness=0)
        self.bottom_frame.grid(row=1, column=0, padx=20, pady=10, sticky="ew")  # Use grid for bottom frame

        # Create general labels for the target and thrusters
        self.target_data_label = Label(self.bottom_frame, text="Target: ", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.target_data_label.grid(row=0, column=0, padx=10, pady=10)

        self.thrusters_data_label = Label(self.bottom_frame, text="Thrusters Data: ", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.thrusters_data_label.grid(row=0, column=1, padx=10, pady=10)

        # Separate labels for Roll, Pitch, and Yaw, positioned at the bottom side by side
        self.roll_label = Label(self.bottom_frame, text="Roll: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.roll_label.grid(row=0, column=2, padx=10, pady=10)

        self.pitch_label = Label(self.bottom_frame, text="Pitch: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.pitch_label.grid(row=0, column=3, padx=10, pady=10)

        self.yaw_label = Label(self.bottom_frame, text="Yaw: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.yaw_label.grid(row=0, column=4, padx=10, pady=10)

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

    def create_rounded_button(self, text, command):
        """Create a button with rounded corners and modern styling."""
        button = Button(self.control_frame, text=text, command=command, bg="#81A1C1", fg="#ECEFF4", font=("Helvetica", 12),
                        bd=0, highlightthickness=0, relief="flat")
        return button

    # Callback functions for button actions
    def move_front(self):
        self.get_logger().info("Moving Front")

    def move_back(self):
        self.get_logger().info("Moving Back")

    def move_left(self):
        self.get_logger().info("Moving Left")

    def move_right(self):
        self.get_logger().info("Moving Right")

    def depth_up(self):
        self.get_logger().info("Increasing Depth")

    def depth_down(self):
        self.get_logger().info("Decreasing Depth")

    def roll_left(self):
        self.get_logger().info("Rolling Left")

    def roll_right(self):
        self.get_logger().info("Rolling Right")

    def pitch_up(self):
        self.get_logger().info("Pitching Up")

    def pitch_down(self):
        self.get_logger().info("Pitching Down")

    def yaw_left(self):
        self.get_logger().info("Yawing Left")

    def yaw_right(self):
        self.get_logger().info("Yawing Right")

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
        pil_image = pil_image.resize(self.get_feed_size())  # Resize dynamically based on the current size

        # Convert the image for Tkinter display
        tk_image = ImageTk.PhotoImage(pil_image)

        # Use `after()` to update the image in the main thread
        self.root.after(0, self.update_front_feed, tk_image)

    def get_feed_size(self):
        # Get the current size of the live feed (width, height)
        return (self.feed_frame.winfo_width(), self.feed_frame.winfo_height())

    def update_sensors_data(self, data):
        # Check if data contains at least 3 elements: roll, pitch, yaw
        if len(data) >= 3:
            roll, pitch, yaw = data[0], data[1], data[2]

            # Update the individual labels with their corresponding values
            self.roll_label.config(text=f"Roll: {roll:.2f}")
            self.pitch_label.config(text=f"Pitch: {pitch:.2f}")
            self.yaw_label.config(text=f"Yaw: {yaw:.2f}")

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
