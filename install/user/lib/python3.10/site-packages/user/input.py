import rclpy
from rclpy.node import Node
import Xlib
import Xlib.display

class KeyboardListenerNode(Node):
    def __init__(self):
        super().__init__('keyboard_listener')
        self.get_logger().info("Keyboard Listener Node Started")

        # Setup Xlib to listen to keyboard input
        self.display = Xlib.display.Display()
        self.root = self.display.screen().root

        # Grab keyboard events
        self.root.grab_keyboard(True,  # grab keyboard (True = grab, False = ungrab)
                                0,      # Grab mode (0 = async, 1 = sync)
                                0,      # Owner events (0 = dont, 1 = allow)
                                0)      # Time (0 = current time)

    def listen_for_keys(self):
        while rclpy.ok():
            # Get events from X server
            event = self.display.next_event()
            if event.type == Xlib.X.KeyPress:
                keycode = event.detail
                keysym = self.display.keycode_to_keysym(keycode, 0)  # Get the symbol of the key
                self.get_logger().info(f"Key pressed: {keysym}")

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardListenerNode()

    try:
        node.listen_for_keys()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
