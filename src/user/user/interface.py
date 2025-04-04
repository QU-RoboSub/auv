import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tkinter import Tk, Frame, Label, Button, Entry, Canvas
from tkinter.ttk import Style
from PIL import Image as PILImage, ImageTk, ImageDraw
import math
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

# Define button style
button_style = {
    'bg': "#81A1C1",
    'fg': "#ECEFF4",
    'font': ("Helvetica", 12),
    'bd': 0,
    'highlightthickness': 0,
    'relief': "flat"
}

class Interface(Node):
    def __init__(self):
        super().__init__('interfaceNode')

        # Add new instance variables
        self.last_sensor_data = []
        self.thruster_enabled = True
        self.thruster_timer = None
        self.logging_active = False

        # Initialize GUI components
        self.root = Tk()
        self.root.title("ROS Interface GUI")
        self.root.geometry("1400x900")
        self.root.configure(bg="#2E3440")

        # Custom style for modern look
        self.style = Style()
        self.style.configure("TFrame", background="#3B4252")
        self.style.configure("TButton", background="#81A1C1", foreground="#ECEFF4", font=("Helvetica", 10), borderwidth=0)
        self.style.map("TButton", background=[("active", "#5E81AC")])


        # Create settings frame first (but don't display it yet)
        self.settings_frame = Frame(self.root, bg="#2E3440")
        self.settings_frame.grid_rowconfigure(0, weight=1)
        self.settings_frame.grid_columnconfigure(0, weight=1)
        self.settings_frame.grid_columnconfigure(1, weight=1)
        
        # Create PID configuration frame
        pid_frame = Frame(self.settings_frame, bg="#3B4252")
        pid_frame.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        # Create container frame for centered content
        pid_container = Frame(pid_frame, bg="#3B4252")
        pid_container.place(relx=0.5, rely=0.5, anchor="center")

        # PID Configuration Title
        Label(pid_container, text="PID Configuration", bg="#3B4252", fg="#ECEFF4", 
            font=("Helvetica", 14, "bold")).grid(row=0, column=0, columnspan=4, pady=10)

        # Column Headers
        headers = ["P", "I", "D"]
        for col, header in enumerate(headers, start=1):
            Label(pid_container, text=header, bg="#3B4252", fg="#81A1C1",
                font=("Helvetica", 12, "bold")).grid(row=1, column=col, padx=5, pady=5)

        # PID Entry fields
        dof_names = ['Depth', 'Roll', 'Pitch', 'Yaw', 'X', 'Y']
        self.pid_entries = []
        for i, name in enumerate(dof_names):
            row = i + 2
            Label(pid_container, text=name, bg="#3B4252", fg="#ECEFF4",
                font=("Helvetica", 12)).grid(row=row, column=0, padx=5, pady=5, sticky="e")

            entries = []
            for col in range(1, 4):
                entry = Entry(pid_container, width=8)
                entry.grid(row=row, column=col, padx=5, pady=5)
                entries.append(entry)
            self.pid_entries.append(tuple(entries))

         # Add separator and offset configuration
        separator = Frame(pid_container, height=2, bg="#4C566A")
        separator.grid(row=8, column=0, columnspan=4, sticky="ew", pady=10)

        Label(pid_container, text="Offsets", bg="#3B4252", fg="#ECEFF4", 
              font=("Helvetica", 14, "bold")).grid(row=9, column=0, columnspan=4, pady=5)

        # Auto-set offsets button
        Button(pid_container, text="Auto Set Offsets", command=self.auto_set_offsets,
               **button_style).grid(row=10, column=0, columnspan=4, pady=5)

        
        # Manual offset entries
        self.offset_entries = []
        offset_dofs = ['Depth', 'Roll', 'Pitch', 'Yaw']

        # Create a container frame for all offset fields
        offset_container = Frame(pid_container, bg="#3B4252")
        offset_container.grid(row=10, column=0, columnspan=4, pady=5, sticky='nsew')

        # Configure column weights for centering
        offset_container.grid_columnconfigure(0, weight=1)

        for i, dof in enumerate(offset_dofs):
            # Create container frame for each row
            row_frame = Frame(offset_container, bg="#3B4252")
            row_frame.grid(row=i, column=0, sticky='ew', pady=2)
            
            # Create inner frame for label+entry pair
            field_frame = Frame(row_frame, bg="#3B4252")
            field_frame.pack(expand=True, fill='x')
            
            # Label with right alignment
            Label(field_frame, text=dof+":", bg="#3B4252", fg="#ECEFF4",
                font=("Helvetica", 12), width=8, anchor='e').pack(side='left', padx=(0, 5))
            
            # Entry field with left alignment
            entry = Entry(field_frame, width=10)
            entry.pack(side='left')
            self.offset_entries.append(entry)


        # Add manual offset set button
        Button(pid_container, text="Set Manual Offsets", command=self.set_manual_offsets,
               **button_style).grid(row=15, column=0, columnspan=4, pady=5)

        # Add first separator above thruster toggle
        Frame(pid_container, height=2, bg="#4C566A").grid(row=16, column=0, columnspan=4, sticky="ew", pady=10)

        # Thruster enable/disable button
        self.thruster_toggle_button = Button(pid_container, text="Disable Thrusters",
                                             command=self.toggle_thrusters, **button_style)
        self.thruster_toggle_button.grid(row=17, column=0, columnspan=4, pady=10)

        # Add second separator below thruster toggle
        Frame(pid_container, height=2, bg="#4C566A").grid(row=18, column=0, columnspan=4, sticky="ew", pady=10)

        # Add logging button
        self.logging_button = Button(pid_container, text="Start Logging", 
                                   command=self.toggle_logging, **button_style)
        self.logging_button.grid(row=19, column=0, columnspan=4, pady=10)

        # Create AUV visualization frame
        self.auv_frame = Frame(self.settings_frame, bg="#3B4252")
        self.auv_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")

        # Create canvas for AUV visualization with adjusted size
        self.auv_canvas = Canvas(self.auv_frame, width=400, height=400, bg="#2E3440", highlightthickness=0)
        self.auv_canvas.pack(pady=10)

        # Compact AUV dimensions
        self.auv_width = 180  # Wider body for horizontal spacing
        self.auv_height = 200  # Shorter body height
        self.x_center = 200
        self.y_center = 200  # Centered in 400x400 canvas
        
        # Main AUV body (shorter rectangle)
        self.auv_body = self.auv_canvas.create_rectangle(
            self.x_center - self.auv_width/2,
            self.y_center - self.auv_height/2,
            self.x_center + self.auv_width/2,
            self.y_center + self.auv_height/2,
            outline="#81A1C1",
            width=3
        )

        # Front/Rear markers moved closer
        self.auv_canvas.create_text(
            self.x_center,
            self.y_center - self.auv_height/2 - 20,
            text="FRONT",
            fill="#81A1C1",
            font=("Helvetica", 12, "bold")
        )
        self.auv_canvas.create_text(
            self.x_center,
            self.y_center + self.auv_height/2 + 20,
            text="REAR",
            fill="#81A1C1",
            font=("Helvetica", 12, "bold")
        )

        # Thruster positions with middle thrusters closer to center
        thruster_positions = {
            # Front-left
            0: (self.x_center - 80, self.y_center - 80, "H"),  # Outer horizontal
            4: (self.x_center - 45, self.y_center - 40, "V"),   # Inner vertical (moved closer)
            
            # Front-right
            1: (self.x_center + 80, self.y_center - 80, "H"),  # Outer horizontal
            5: (self.x_center + 45, self.y_center - 40, "V"),   # Inner vertical (moved closer)
            
            # Rear-left
            2: (self.x_center - 80, self.y_center + 80, "H"),  # Outer horizontal
            6: (self.x_center - 45, self.y_center + 40, "V"),   # Inner vertical (moved closer)
            
            # Rear-right
            3: (self.x_center + 80, self.y_center + 80, "H"),  # Outer horizontal
            7: (self.x_center + 45, self.y_center + 40, "V")    # Inner vertical (moved closer)
        }

        # Draw thrusters with adjusted positions
        self.thruster_labels = {}
        for thruster_id, (x, y, type) in thruster_positions.items():
            color = "#8FBCBB" if type == "H" else "#88C0D0"
            # Thruster body
            self.auv_canvas.create_oval(x-12, y-12, x+12, y+12, 
                                    fill=color, outline="#ECEFF4", width=1)
            
            # Value labels (maintain same label distance from thrusters)
            label = self.auv_canvas.create_text(
                x,
                y + 30,  # Same offset as before
                text="0.00",
                fill="#ECEFF4",
                font=("Helvetica", 10, "bold")
            )
            self.thruster_labels[thruster_id] = label

                # Create graph frame under AUV visualization
        self.graph_frame = Frame(self.auv_frame, bg="#2E3440")
        self.graph_frame.pack(fill='both', expand=True, pady=(10, 0))
        
        # Initialize data buffers
        self.time_buffer = np.linspace(0, 100, 100)
        self.data_buffers = {
            'depth': {'actual': np.zeros(100), 'target': np.zeros(100)},
            'roll': {'actual': np.zeros(100), 'target': np.zeros(100)},
            'pitch': {'actual': np.zeros(100), 'target': np.zeros(100)},
            'yaw': {'actual': np.zeros(100), 'target': np.zeros(100)}
        }
        
        # Create graphs
        self.figures = []
        self.axes = []
        self.lines = []
        plt.style.use('fast')

        # In the figure creation:


        
        dofs = ['Depth', 'Roll', 'Pitch', 'Yaw']
        colors = ['#81A1C1', '#A3BE8C']
        
        for i, dof in enumerate(dofs):
            frame = Frame(self.graph_frame, bg="#2E3440")
            frame.grid(row=i//2, column=i%2, padx=5, pady=5, sticky='nsew')
            
            fig = Figure(figsize=(3, 1.5), dpi=80, facecolor="#2E3440")
            fig.set_tight_layout(True)
            ax = fig.add_subplot(111, xmargin=0, ymargin=0)
            ax.set_facecolor("#3B4252")
            ax.tick_params(axis='x', colors='#ECEFF4')
            ax.tick_params(axis='y', colors='#ECEFF4')
            ax.set_title(f"{dof} Tracking", color='#ECEFF4')
            ax.set_xlabel('Time', color='#ECEFF4')
            ax.set_ylabel('Value', color='#ECEFF4')
            ax.set_autoscale_on(False)  # Disable auto-scaling for performance
            
            line_actual, = ax.plot([], [], color=colors[0], label='Actual')
            line_target, = ax.plot([], [], color=colors[1], label='Target')
            ax.legend(facecolor="#3B4252", edgecolor="#3B4252", labelcolor="#ECEFF4")
            
            canvas = FigureCanvasTkAgg(fig, master=frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill='both', expand=True)
            
            self.figures.append(fig)
            self.axes.append(ax)
            self.lines.append((line_actual, line_target))

        # Configure grid layout
        self.graph_frame.grid_rowconfigure(0, weight=1)
        self.graph_frame.grid_rowconfigure(1, weight=1)
        self.graph_frame.grid_columnconfigure(0, weight=1)
        self.graph_frame.grid_columnconfigure(1, weight=1)
        
        # Add update timer for graphs
        self.graph_timer = self.create_timer(0.0025, self.update_graphs)

        # Create button frame at bottom right of settings window
        button_frame = Frame(self.settings_frame, bg="#2E3440")
        button_frame.grid(row=1, column=0, columnspan=2, sticky="se", padx=20, pady=20)

        # Add Save and Back buttons
        Button(button_frame, text="Save", command=self.save_pid_settings, 
               **button_style).pack(side="right", padx=10, ipadx=15)
        Button(button_frame, text="Back", command=self.show_main, 
               **button_style).pack(side="right", padx=10, ipadx=15)


        # Create main container frame
        self.main_frame = Frame(self.root, bg="#2E3440")
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)

        # Create header frame with settings button
        self.header_frame = Frame(self.main_frame, bg="#2E3440")
        self.header_frame.pack(fill="x", pady=(0, 10))

        # Create gear icon button in top right
        self.gear_icon = self.create_gear_icon()
        self.settings_button = Button(self.header_frame,
                                    image=self.gear_icon,
                                    bg="#81A1C1",
                                    activebackground="#5E81AC",
                                    bd=0,
                                    highlightthickness=0,
                                    relief="flat",
                                    command=self.show_settings)
        self.settings_button.pack(side="right", padx=10, pady=5)

        # Create content container frame
        self.content_frame = Frame(self.main_frame, bg="#2E3440")
        self.content_frame.pack(fill="both", expand=True)

        # Create a frame for the live feed
        self.feed_frame = Frame(self.content_frame, bg="#3B4252", bd=0, highlightthickness=0)
        self.feed_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))

        # Create a label for the front feed
        self.front_feed_label = Label(self.feed_frame, bg="#3B4252")
        self.front_feed_label.pack(fill="both", expand=True)

        # Create a frame for the DOF control buttons
        self.control_frame = Frame(self.content_frame, bg="#3B4252", bd=0, highlightthickness=0)
        self.control_frame.pack(side="right", fill="y", padx=(10, 0))

        # Create buttons for each degree of freedom
        self.front_button = self.create_rounded_button("▲")
        self.front_button.grid(row=0, column=1, padx=5, pady=5)
        self.front_button.bind('<ButtonPress-1>', lambda e: self.on_press(5, 1.0))
        self.front_button.bind('<ButtonRelease-1>', lambda e: self.on_release(5))

        self.back_button = self.create_rounded_button("▼")
        self.back_button.grid(row=2, column=1, padx=5, pady=5)
        self.back_button.bind('<ButtonPress-1>', lambda e: self.on_press(5, -1.0))
        self.back_button.bind('<ButtonRelease-1>', lambda e: self.on_release(5))

        self.left_button = self.create_rounded_button("◄")
        self.left_button.grid(row=1, column=0, padx=5, pady=5)
        self.left_button.bind('<ButtonPress-1>', lambda e: self.on_press(4, -1.0))
        self.left_button.bind('<ButtonRelease-1>', lambda e: self.on_release(4))

        self.right_button = self.create_rounded_button("►")
        self.right_button.grid(row=1, column=2, padx=5, pady=5)
        self.right_button.bind('<ButtonPress-1>', lambda e: self.on_press(4, 1.0))
        self.right_button.bind('<ButtonRelease-1>', lambda e: self.on_release(4))

        self.depth_up_button = self.create_rounded_button("Depth ▲")
        self.depth_up_button.grid(row=3, column=1, padx=5, pady=5)
        self.depth_up_button.bind('<ButtonPress-1>', lambda e: self.on_press(0, 1.0))
        self.depth_up_button.bind('<ButtonRelease-1>', lambda e: self.on_release(0))

        self.depth_down_button = self.create_rounded_button("Depth ▼")
        self.depth_down_button.grid(row=4, column=1, padx=5, pady=5)
        self.depth_down_button.bind('<ButtonPress-1>', lambda e: self.on_press(0, -1.0))
        self.depth_down_button.bind('<ButtonRelease-1>', lambda e: self.on_release(0))

        self.roll_left_button = self.create_rounded_button("Roll ◄")
        self.roll_left_button.grid(row=5, column=0, padx=5, pady=5)
        self.roll_left_button.bind('<ButtonPress-1>', lambda e: self.on_press(1, -1.0))
        self.roll_left_button.bind('<ButtonRelease-1>', lambda e: self.on_release(1))

        self.roll_right_button = self.create_rounded_button("Roll ►")
        self.roll_right_button.grid(row=5, column=2, padx=5, pady=5)
        self.roll_right_button.bind('<ButtonPress-1>', lambda e: self.on_press(1, 1.0))
        self.roll_right_button.bind('<ButtonRelease-1>', lambda e: self.on_release(1))

        self.pitch_up_button = self.create_rounded_button("Pitch ▲")
        self.pitch_up_button.grid(row=6, column=1, padx=5, pady=5)
        self.pitch_up_button.bind('<ButtonPress-1>', lambda e: self.on_press(2, 1.0))
        self.pitch_up_button.bind('<ButtonRelease-1>', lambda e: self.on_release(2))

        self.pitch_down_button = self.create_rounded_button("Pitch ▼")
        self.pitch_down_button.grid(row=7, column=1, padx=5, pady=5)
        self.pitch_down_button.bind('<ButtonPress-1>', lambda e: self.on_press(2, -1.0))
        self.pitch_down_button.bind('<ButtonRelease-1>', lambda e: self.on_release(2))

        self.yaw_left_button = self.create_rounded_button("Yaw ◄")
        self.yaw_left_button.grid(row=8, column=0, padx=5, pady=5)
        self.yaw_left_button.bind('<ButtonPress-1>', lambda e: self.on_press(3, -1.0))
        self.yaw_left_button.bind('<ButtonRelease-1>', lambda e: self.on_release(3))

        self.yaw_right_button = self.create_rounded_button("Yaw ►")
        self.yaw_right_button.grid(row=8, column=2, padx=5, pady=5)
        self.yaw_right_button.bind('<ButtonPress-1>', lambda e: self.on_press(3, 1.0))
        self.yaw_right_button.bind('<ButtonRelease-1>', lambda e: self.on_release(3))

        # Add Target Value Entry Fields
          # Add Target Value Entry Fields
        Label(self.control_frame, text="Set Targets", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12)).grid(row=9, column=0, columnspan=3, pady=(10, 5))

        self.target_entries = []
        dof_targets = ['Depth', 'Roll', 'Pitch', 'Yaw', 'X', 'Y']
        for i, dof in enumerate(dof_targets):
            row = 10 + i
            Label(self.control_frame, text=f"{dof}:", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 10)).grid(row=row, column=0, padx=5, pady=2, sticky='e')
            entry = Entry(self.control_frame, width=10)
            entry.grid(row=row, column=1, columnspan=2, padx=5, pady=2, sticky='w')
            self.target_entries.append(entry)

        self.set_target_button = Button(self.control_frame, text="Update Targets", command=self.set_targets, **button_style)
        self.set_target_button.grid(row=16, column=0, columnspan=3, pady=(5,2))

        # Add mini AUV visualization under target inputs
        self.mini_auv_frame = Frame(self.control_frame, bg="#3B4252")
        self.mini_auv_frame.grid(row=17, column=0, columnspan=3, pady=(5, 0))

        self.mini_auv_canvas = Canvas(self.mini_auv_frame, width=250, height=180, bg="#2E3440", highlightthickness=0)
        self.mini_auv_canvas.pack(pady=0)

        # Mini AUV dimensions
        mini_auv_width = 140
        mini_auv_height = 110
        mini_x_center = 130
        mini_y_center = 90

        # Draw mini AUV body
        self.mini_auv_body = self.mini_auv_canvas.create_rectangle(
            mini_x_center - mini_auv_width/2,
            mini_y_center - mini_auv_height/2,
            mini_x_center + mini_auv_width/2,
            mini_y_center + mini_auv_height/2,
            outline="#81A1C1",
            width=2
        )

        # Front/Rear markers
        self.mini_auv_canvas.create_text(
            mini_x_center,
            mini_y_center - mini_auv_height/2 - 10,
            text="FRONT",
            fill="#81A1C1",
            font=("Helvetica", 10, "bold")
        )
        self.mini_auv_canvas.create_text(
            mini_x_center,
            mini_y_center + mini_auv_height/2 + 10,
            text="REAR",
            fill="#81A1C1",
            font=("Helvetica", 10, "bold")
        )

        # Update thruster positions with more spacing
        mini_thruster_positions = {
            0: (mini_x_center - 55, mini_y_center - 45, "H"),  # Increased from 50
            4: (mini_x_center - 30, mini_y_center - 25, "V"),  # Moved outward
            1: (mini_x_center + 55, mini_y_center - 45, "H"),
            5: (mini_x_center + 30, mini_y_center - 25, "V"),
            2: (mini_x_center - 55, mini_y_center + 45, "H"),
            6: (mini_x_center - 30, mini_y_center + 25, "V"),
            3: (mini_x_center + 55, mini_y_center + 45, "H"),
            7: (mini_x_center + 30, mini_y_center + 25, "V")
        }

        # Draw mini thrusters
        self.mini_thruster_labels = {}
        for thruster_id, (x, y, type) in mini_thruster_positions.items():
            color = "#8FBCBB" if type == "H" else "#88C0D0"
            # Thruster body
            self.mini_auv_canvas.create_oval(x-8, y-8, x+8, y+8, 
                                        fill=color, outline="#ECEFF4", width=1)
            
            # Value labels
            label = self.mini_auv_canvas.create_text(
                x,
                y + 18,  # Smaller offset
                text="0.00",
                fill="#ECEFF4",
                font=("Helvetica", 9, "bold")
            )
            self.mini_thruster_labels[thruster_id] = label

        # Create a frame for the bottom labels
        self.bottom_frame = Frame(self.main_frame, bg="#3B4252", bd=0, highlightthickness=0)
        self.bottom_frame.pack(fill="x", pady=(10, 0))

        # Labels for target and orientation data
        self.target_data_label = Label(self.bottom_frame, text="Target: ", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.target_data_label.pack(side="left", padx=10, pady=10)

        self.roll_label = Label(self.bottom_frame, text="Roll: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.roll_label.pack(side="left", padx=10, pady=10)

        self.pitch_label = Label(self.bottom_frame, text="Pitch: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.pitch_label.pack(side="left", padx=10, pady=10)

        self.yaw_label = Label(self.bottom_frame, text="Yaw: N/A", bg="#3B4252", fg="#ECEFF4", font=("Helvetica", 12))
        self.yaw_label.pack(side="left", padx=10, pady=10)

        # Initialize ROS components
        self.pid_publisher = self.create_publisher(Float32MultiArray, 'pid_update', 10)
        self.sensors_subscription = self.create_subscription(Float32MultiArray, 'sensors', self.listener_callback_sensors, 10)
        self.target_subscription = self.create_subscription(String, 'target', self.listener_callback_target, 10)
        self.thrusters_subscription = self.create_subscription(Float32MultiArray, 'thrusters', self.listener_callback_thrusters, 10)
        self.frontFeed_subscription = self.create_subscription(Image, 'frontFeed', self.listener_callback_frontFeed, 10)
        self.publisher_userIn = self.create_publisher(Float32MultiArray, 'userIn', 10)
        self.target_publisher = self.create_publisher(Float32MultiArray, 'target_update', 10)
        self.thruster_publisher = self.create_publisher(Float32MultiArray, 'thrusters', 10)
        self.offset_publisher = self.create_publisher(Float32MultiArray, 'offset_update', 10)
        self.userIn_state = [0.0] * 6
        self.thruster_values = [0.0] * 8
        self.bridge = CvBridge()

        self.get_logger().info('Interface Node started')

    def create_gear_icon(self):
        """Create a gear icon with shorter teeth"""
        icon_size = (32, 32)
        gear_icon = PILImage.new("RGBA", icon_size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(gear_icon)
        
        # Main gear circle
        draw.ellipse((8, 8, 24, 24), outline="#ECEFF4", width=2)
        
        # Draw shorter gear teeth
        tooth_length = 3
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            inner_x = 16 + 11 * math.cos(rad)
            inner_y = 16 + 11 * math.sin(rad)
            outer_x = 16 + (11 + tooth_length) * math.cos(rad)
            outer_y = 16 + (11 + tooth_length) * math.sin(rad)
            draw.line([(inner_x, inner_y), (outer_x, outer_y)], fill="#ECEFF4", width=2)
        
        # Inner circle
        draw.ellipse((12, 12, 20, 20), outline="#ECEFF4", width=2)
        
        # Center cross
        draw.line((16, 14, 16, 18), fill="#ECEFF4", width=2)
        draw.line((14, 16, 18, 16), fill="#ECEFF4", width=2)
        
        return ImageTk.PhotoImage(gear_icon)
    
    def get_thruster_location(self, thruster_id):
        """Helper method to identify thruster position"""
        locations = {
            0: "front-left-outer",
            4: "front-left-inner",
            1: "front-right-outer",
            5: "front-right-inner",
            2: "rear-left-outer",
            6: "rear-left-inner",
            3: "rear-right-outer",
            7: "rear-right-inner"
        }
        return locations.get(thruster_id, "unknown")

    def toggle_thrusters(self):
        """Toggle between PID control and manual thruster override"""
        self.thruster_enabled = not self.thruster_enabled
        if self.thruster_enabled:
            self.thruster_toggle_button.config(text="Disable Thrusters")
            if self.thruster_timer:
                self.thruster_timer.cancel()
                self.thruster_timer = None
        else:
            self.thruster_toggle_button.config(text="Enable Thrusters")
            self.thruster_timer = self.create_timer(0.1, self.publish_thruster_override)

    def publish_thruster_override(self):
        """Publish 7.5 to all thrusters when disabled"""
        if not self.thruster_enabled:
            msg = Float32MultiArray()
            msg.data = [7.5] * 8
            self.thruster_publisher.publish(msg)

    def auto_set_offsets(self):
        """Auto-populate offset entries with current sensor values"""
        try:
            # Assuming sensor data order: [roll, pitch, yaw, depth]
            if len(self.last_sensor_data) >= 4:
                depth = self.last_sensor_data[3]
                roll = self.last_sensor_data[0]
                pitch = self.last_sensor_data[1]
                yaw = self.last_sensor_data[2]
                
                offsets = [depth, roll, pitch, yaw]
                for entry, value in zip(self.offset_entries, offsets):
                    entry.delete(0, 'end')
                    entry.insert(0, f"{value:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error setting offsets: {str(e)}")

    def update_thruster_visuals(self, canvas, thruster_labels):
        """Helper function to update thruster visuals for any canvas"""
        for thruster_id, label_id in thruster_labels.items():
            value = self.thruster_values[thruster_id]
            canvas.itemconfig(label_id, text=f"{value:.2f}")
            
            # Determine color based on thrust direction and type
            location = self.get_thruster_location(thruster_id)
            if "outer" in location:
                base_color = "#8FBCBB"  # Horizontal thrusters
            else:
                base_color = "#88C0D0"  # Vertical thrusters

            if value > 0:
                color = "#A3BE8C" if "inner" in location else "#8FBCBB"
            elif value < 0:
                color = "#BF616A"
            else:
                color = base_color
            
            # Update thruster circle color (assuming circle is label_id - 1)
            canvas.itemconfigure(label_id - 1, fill=color)

    def update_thruster_display(self):
        """Update both AUV visualizations"""
        self.update_thruster_visuals(self.auv_canvas, self.thruster_labels)
        self.update_thruster_visuals(self.mini_auv_canvas, self.mini_thruster_labels)

    def show_settings(self):
        """Switch to the settings frame."""
        self.main_frame.pack_forget()
        self.settings_frame.place(x=0, y=0, relwidth=1, relheight=1)

    def show_main(self):
        """Switch back to the main frame."""
        self.settings_frame.place_forget()
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)

    def save_pid_settings(self):
        """Save PID values from the settings frame."""
        pid_values = []
        for p_entry, i_entry, d_entry in self.pid_entries:
            try:
                pid_values.append(float(p_entry.get()))
                pid_values.append(float(i_entry.get()))
                pid_values.append(float(d_entry.get()))
            except ValueError:
                self.get_logger().error("Invalid PID value entered")
            try:
                offset_values = [float(entry.get()) for entry in self.offset_entries]
                offset_msg = Float32MultiArray()
                offset_msg.data = offset_values
                self.offset_publisher.publish(offset_msg)  # Add this publisher in __init__
            except ValueError:
                self.get_logger().error("Invalid offset value entered")
                return

        msg = Float32MultiArray()
        msg.data = pid_values
        self.pid_publisher.publish(msg)
        self.show_main()

    def set_targets(self):
        """Read target entries and publish them."""
        try:
            targets = [float(entry.get()) for entry in self.target_entries]
        except ValueError:
            self.get_logger().error("Invalid target value entered")
            return
        
        msg = Float32MultiArray()
        msg.data = targets
        self.target_publisher.publish(msg)
        self.get_logger().info(f'Published new targets: {targets}')

    def create_rounded_button(self, text):
        """Create a button with rounded corners and modern styling."""
        button = Button(self.control_frame, text=text, **button_style)
        return button

    def on_press(self, index, value):
        """Handle button press event by updating the state array and publishing."""
        self.userIn_state[index] = value
        self.publish_userIn_state()

    def on_release(self, index):
        """Handle button release event by resetting the state array and publishing."""
        self.userIn_state[index] = 0.0
        self.publish_userIn_state()

    def publish_userIn_state(self):
        """Publish the current user input state as a Float32MultiArray."""
        msg = Float32MultiArray()
        msg.data = self.userIn_state
        self.publisher_userIn.publish(msg)
        self.get_logger().debug(f'Publishing userIn: {msg.data}')

    def listener_callback_sensors(self, msg):
        self.root.after(0, self.update_sensors_data, msg.data)

    def listener_callback_target(self, msg):
        self.root.after(0, self.update_target_data, msg.data)

    def listener_callback_thrusters(self, msg):
        self.thruster_values = list(msg.data)
        self.root.after(0, self.update_thruster_display)

    def listener_callback_frontFeed(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        pil_image = PILImage.fromarray(cv_image)
        pil_image = pil_image.resize((self.feed_frame.winfo_width(), self.feed_frame.winfo_height()))
        tk_image = ImageTk.PhotoImage(pil_image)
        self.root.after(0, self.update_front_feed, tk_image)

    def update_sensors_data(self, data):
        self.last_sensor_data = data
        if len(data) >= 3:
            self.roll_label.config(text=f"Roll: {data[0]:.2f}")
            self.pitch_label.config(text=f"Pitch: {data[1]:.2f}")
            self.yaw_label.config(text=f"Yaw: {data[2]:.2f}")

    def update_target_data(self, data):
        self.target_data_label.config(text=f"Target: {data}")

    def update_front_feed(self, tk_image):
        self.front_feed_label.config(image=tk_image)
        self.front_feed_label.image = tk_image

    def toggle_logging(self):
        """Toggle logging state"""
        self.logging_active = not self.logging_active
        if self.logging_active:
            self.logging_button.config(text="Stop Logging")
            self.start_logging()
        else:
            self.logging_button.config(text="Start Logging")
            self.stop_logging()

    def start_logging(self):
        """Implement logging initialization here"""
        self.get_logger().info("Logging started")
        # Add your logging initialization code

    def stop_logging(self):
        """Implement logging cleanup here"""
        self.get_logger().info("Logging stopped")
        # Add your logging cleanup code

    def set_manual_offsets(self):
        """Handle manual offset setting"""
        try:
            offset_values = [float(entry.get()) for entry in self.offset_entries]
            msg = Float32MultiArray()
            msg.data = offset_values
            self.offset_publisher.publish(msg)  # Make sure to create this publisher in __init__
            self.get_logger().info(f'Published manual offsets: {offset_values}')
        except ValueError:
            self.get_logger().error("Invalid offset value entered")

    def update_graphs(self):
        """Update all graphs with new data"""
        for i, dof in enumerate(['depth', 'roll', 'pitch', 'yaw']):
            actual_line, target_line = self.lines[i]
            
            # Update data
            actual_line.set_data(self.time_buffer, self.data_buffers[dof]['actual'])
            target_line.set_data(self.time_buffer, self.data_buffers[dof]['target'])
            
            # Update axis limits
            min_val = min(np.min(self.data_buffers[dof]['actual']), 
                        np.min(self.data_buffers[dof]['target']))
            max_val = max(np.max(self.data_buffers[dof]['actual']), 
                        np.max(self.data_buffers[dof]['target']))
            self.axes[i].set_ylim(min_val - 0.1, max_val + 0.1)
            self.axes[i].set_xlim(self.time_buffer[0], self.time_buffer[-1])
            
            self.figures[i].canvas.draw_idle()

    # Update the data buffer management:
    def update_data_buffer(self, actual_values, target_values):
        """Update data buffers with new values"""
        # Shift time buffer at 400Hz resolution
        self.time_buffer = np.roll(self.time_buffer, -1)
        self.time_buffer[-1] = self.time_buffer[-2] + 0.0025  # 400Hz increment
        
        # Update actual values
        self.data_buffers['depth']['actual'] = np.roll(self.data_buffers['depth']['actual'], -1)
        self.data_buffers['roll']['actual'] = np.roll(self.data_buffers['roll']['actual'], -1)
        self.data_buffers['pitch']['actual'] = np.roll(self.data_buffers['pitch']['actual'], -1)
        self.data_buffers['yaw']['actual'] = np.roll(self.data_buffers['yaw']['actual'], -1)
        
        self.data_buffers['depth']['actual'][-1] = actual_values[0]
        self.data_buffers['roll']['actual'][-1] = actual_values[1]
        self.data_buffers['pitch']['actual'][-1] = actual_values[2]
        self.data_buffers['yaw']['actual'][-1] = actual_values[3]
        
        # Update target values
        self.data_buffers['depth']['target'] = np.roll(self.data_buffers['depth']['target'], -1)
        self.data_buffers['roll']['target'] = np.roll(self.data_buffers['roll']['target'], -1)
        self.data_buffers['pitch']['target'] = np.roll(self.data_buffers['pitch']['target'], -1)
        self.data_buffers['yaw']['target'] = np.roll(self.data_buffers['yaw']['target'], -1)
        
        self.data_buffers['depth']['target'][-1] = target_values[0]
        self.data_buffers['roll']['target'][-1] = target_values[1]
        self.data_buffers['pitch']['target'][-1] = target_values[2]
        self.data_buffers['yaw']['target'][-1] = target_values[3]

    def run_gui(self):
        self.root.mainloop()

    def destroy_node(self):
        self.root.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    interfaceNode = Interface()

    def ros_spin():
        rclpy.spin(interfaceNode)

    import threading
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    interfaceNode.run_gui()
    interfaceNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()