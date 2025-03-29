import rclpy
from rclpy.node import Node
import numpy as np
import math as m
import serial
import time
import csv
from pathlib import Path
import json

from std_msgs.msg import String, Float32MultiArray
from . import pid as PID

# Constants
# Globals
isDisabled = True
isHoverMode = False  # New global variable for hover mode
FORWARD_THRESHOLD_MM = 20
current_log_file = "data_log.csv"  # Default log file name
isROVMode = False
ROV_DEVIATIONS = {
    'Depth': 0.0,
    'Roll': 0.0,
    'Pitch': 0.0,
    'Yaw': 0.0,
    'X': 0.0
}

# Add these constants near the top
CONFIG_FILE = Path.cwd() / "logs" / "config.json"
DEFAULT_LOG_FILE = Path.cwd() / "logs" / "data_log.csv"
last_yaw_a = 0

FORWARD_THRESHOLD_MM = 20

class Dof():
    """
    Degree of Freedom class for managing control parameters
    """
    def __init__(self, name, pid=None, t=0, offset=0, err=0):
        self.name = name
        self.pid = pid if pid is not None else [0, 0, 0]
        self.t = t  # Target value
        self.offset = offset
        self.error = err
        self.a = 0  # Current actual value


def trans_motion(pwr_x, pwr_y, arr):
    """
    Apply translational motion to thruster array
    """
    if pwr_x >= -2 and pwr_x <= 2 and pwr_y >= -2 and pwr_y <= 2:
        arr[[3, 4]] = arr[[3, 4]] + pwr_x
        arr[[0, 7]] = arr[[0, 7]] - pwr_x
        arr[[0, 4]] = arr[[0, 4]] + pwr_y
        arr[[3, 7]] = arr[[3, 7]] - pwr_y
    else:
        raise ValueError("Each input must be between -2 and 2")
    return arr

def get_valid_int_input(prompt, min_val=None, max_val=None):
    """Get validated integer input within range"""
    while True:
        try:
            value = int(input(prompt))
            if min_val is not None and value < min_val:
                print(f"Value must be at least {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"Value must be at most {max_val}")
                continue
            return value
        except ValueError:
            print("Please enter a valid number")

def get_valid_float_input(prompt, min_val=None, max_val=None):
    """Get validated float input within range"""
    while True:
        try:
            value = float(input(prompt))
            if min_val is not None and value < min_val:
                print(f"Value must be at least {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"Value must be at most {max_val}")
                continue
            return value
        except ValueError:
            print("Please enter a valid number")

def validate_date(date_str):
    """Validate date string format YYYY-MM-DD"""
    try:
        time.strptime(date_str, "%Y-%m-%d")
        return True
    except ValueError:
        return False

def validate_time(time_str):
    """Validate time string format HH:MM"""
    try:
        time.strptime(time_str, "%H:%M")
        return True
    except ValueError:
        return False
    

def log_data(sensor_data, thruster_output, dof_list, hover_mode=False, filename=None):
    """Modified to handle file creation/appending with better error handling"""
    global current_log_file
    try:
        if filename is None:
            filename = current_log_file

        # Ensure we have an absolute path and the directory exists
        log_path = Path(filename).absolute()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        sensor_data = list(sensor_data)
        thruster_output = list(thruster_output)
        thrusterConf = (27, 26, 25, 33, 32, 5, 18, 19)
        pid = ("P", "I", "D")

        targets = [dof.t for dof in dof_list]
        offsets = [dof.offset for dof in dof_list]

        pid_values = []
        for dof in dof_list:
            pid_values.extend(dof.pid)

        # Create header row
        header = (
            ["Timestamp"] +
            [f"{dof.name}" for dof in dof_list] +
            [f"Thruster {i}" for i in thrusterConf] +
            [f"Target {dof.name}" for dof in dof_list] +
            [f"Offset {dof.name}" for dof in dof_list] +
            [f"{dof_list[i].name} {j}" for i in range(0, 6) for j in pid] +
            ["Hover Mode"]
        )

        # Create data row
        row = (
            [timestamp] +
            sensor_data +
            thruster_output +
            targets +
            offsets +
            pid_values +
            [hover_mode]
        )

        # Check if file is empty (new file or no content)
        is_empty = not log_path.exists() or log_path.stat().st_size == 0

        with open(log_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if is_empty:
                writer.writerow(header)
            writer.writerow(row)
            file.flush()
            
    except Exception as e:
        print(f"Error logging data: {str(e)}")
        try:
            fallback_path = Path.cwd() / "logs" / "fallback_log.csv"
            is_empty = not fallback_path.exists() or fallback_path.stat().st_size == 0
            
            fallback_path.parent.mkdir(exist_ok=True)
            with open(fallback_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                if is_empty:
                    writer.writerow(header)
                writer.writerow(row)
                file.flush()
        except Exception as e2:
            print(f"Failed to write to fallback log: {str(e2)}")

def load_last_log_file():
    """Load the last used log file path from config"""
    try:
        if CONFIG_FILE.exists():
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
                last_file = Path(config.get('last_log_file', str(DEFAULT_LOG_FILE)))
                if last_file.exists():
                    return last_file
    except Exception as e:
        print(f"Error loading last log file: {str(e)}")
    return DEFAULT_LOG_FILE

def save_log_file_config(log_file):
    """Save the current log file path to config"""
    try:
        CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
        with open(CONFIG_FILE, 'w') as f:
            json.dump({'last_log_file': str(log_file)}, f)
    except Exception as e:
        print(f"Error saving log file config: {str(e)}")

def create_new_log():
    """Create a new log file with custom identifier and ensure directory exists"""
    global current_log_file
    try:
        # Create logs directory if it doesn't exist
        log_dir = Path.cwd() / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        
        print("\nCreate new log file:")
        print("1) Use custom identifier")
        print("2) Use manual date/time")
        print("3) Back")
        
        choice = get_valid_int_input("Choice: ", 1, 3)
        
        match choice:
            case 1:
                while True:
                    identifier = input("Enter identifier for log file: ").strip()
                    if identifier and identifier.replace("_", "").isalnum():
                        current_log_file = log_dir / f"data_log_{identifier}.csv"
                        print(f"New log file: {current_log_file}")
                        break
                    print("Please enter a valid identifier (letters, numbers, and underscores only)")
            case 2:
                while True:
                    date = input("Enter date (YYYY-MM-DD): ")
                    if validate_date(date):
                        break
                    print("Invalid date format. Please use YYYY-MM-DD")
                
                while True:
                    time_str = input("Enter time (HH:MM): ")
                    if validate_time(time_str):
                        break
                    print("Invalid time format. Please use HH:MM")
                
                current_log_file = log_dir / f"data_log_{date}_{time_str}.csv"
                print(f"New log file: {current_log_file}")
            case 3:
                return

        # Test write access
        try:
            with open(current_log_file, 'a') as f:
                f.write("")
            print(f"Logging will continue in: {current_log_file}")
            save_log_file_config(current_log_file)  # Save the new log file path
        except Exception as e:
            print(f"Warning: Could not write to log file: {str(e)}")
            # Try to use home directory as fallback
            home_dir = Path.home() / "auv_logs"
            home_dir.mkdir(parents=True, exist_ok=True)
            current_log_file = home_dir / current_log_file.name
            print(f"Using alternate location: {current_log_file}")
            save_log_file_config(current_log_file)  # Save the fallback path
            
    except Exception as e:
        print(f"Error creating log file: {str(e)}")
        # Use a simple timestamped filename as last resort
        current_log_file = Path.cwd() / f"auv_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        print(f"Using fallback log file: {current_log_file}")
        save_log_file_config(current_log_file)  # Save the fallback path



def mainMenu(node):
    global isDisabled, isHoverMode, isROVMode
    while True:
        try:
            print("\n=== Main Menu ===")
            # Show status of all modes
            print("=== Status ===")
            print(f"Thrusters: {'DISABLED' if isDisabled else 'ENABLED'}")
            if isROVMode:
                print("Mode: ROV CONTROL")
            elif isHoverMode:
                print(f"Mode: HOVER {'(maintaining position)' if isHoverMode else '(following targets)'}")
            else:
                print("Mode: NORMAL")
            print("\n=== Menu Options ===")
            
            # Format menu options with status
            print("1) Set Offset")
            print("2) PID")
            print("3) Target Values")
            print(f"4) Thrusters [{'DISABLED' if isDisabled else 'ENABLED'}]")
            print("5) Reset Values")
            print("6) Save Values to CSV")
            print(f"7) Hover Mode [{'ON' if isHoverMode else 'OFF'}]" + 
                    f"{' (maintaining position)' if isHoverMode else ' (following targets)'}")
            print("8) New Log File")
            print(f"9) ROV Mode [{'ON' if isROVMode else 'OFF'}]\n")
            
            choice = get_valid_int_input("Choice: ", 1, 9)
            
            match choice:
                case 1:
                    setOffset(node.dof, node.sensorsData)
                    print(f'New offset values\nDepth: {node.dof[0].offset:.2f}\nRoll: {node.dof[1].offset:.2f}')
                    print(f'Pitch: {node.dof[2].offset:.2f}\nYaw: {node.dof[3].offset:.2f}')
                    print(f'X: {node.dof[4].offset:.2f}\nY: {node.dof[5].offset:.2f}\n')
                case 2:
                    pidMenu(node)
                case 3:
                    targetMenu(node)
                case 4:
                    isDisabled = not isDisabled
                    node.disableThruster(isDisabled)
                    print(f"Thrusters {'disabled' if isDisabled else 'enabled'}")
                case 5:
                    while True:
                        c = input("Are you sure you want to reset all values? [y/N]: ").lower()
                        if c in ['y', 'n', '']:
                            if c == 'y':
                                resetValues(node.dof)
                            break
                        print("Please enter 'y' or 'n'")
                case 6:
                    save_to_csv(node.dof)
                    print("Data saved to CSV file.")
                case 7:
                    isHoverMode = not isHoverMode
                    print(f"Hover mode {'enabled' if isHoverMode else 'disabled'}")
                    if isHoverMode:
                        node.dof[4].t = 0  # X target
                        node.dof[3].t = 0  # Yaw target
                case 8:
                    create_new_log()
                # case 9:
                #     isROVMode = not isROVMode
                #     if isROVMode:
                #         isHoverMode = False  # Disable hover mode when entering ROV mode
                #         print("ROV Mode enabled")
                #         rovMenu(node)
                #     else:
                #         # Zero all deviations when exiting ROV mode
                #         for key in ROV_DEVIATIONS:
                #             ROV_DEVIATIONS[key] = 0.0
                #         print("ROV Mode disabled")
        except Exception as e:
            print(f"Error in menu: {str(e)}")

def pidMenu(node):
    while True:
        print("\n=== PID Menu ===")
        for i, dof in enumerate(node.dof, 1):
            print(f"{i}) {dof.name}")
        print("7) Back\n")
        
        choice = get_valid_int_input("Choice: ", 1, 7)
        if choice == 7:
            return
            
        i = choice - 1
        print(f"\nCurrent PID Values for {node.dof[i].name}")
        print(f"P= {node.dof[i].pid[0]:.3f}, I= {node.dof[i].pid[1]:.3f}, D= {node.dof[i].pid[2]:.3f}\n")
        print("1) Change P \n2) Change I\n3) Change D\n4) Back to main\n")
        
        sub_choice = get_valid_int_input("Choice: ", 1, 4)
        if sub_choice == 4:
            return
            
        value = get_valid_float_input(f"Enter new {'P' if sub_choice == 1 else 'I' if sub_choice == 2 else 'D'} value: ")
        node.dof[i].pid[sub_choice - 1] = value

def targetMenu(node):
    while True:
        print("\n=== Target Values ===")
        for i, dof in enumerate(node.dof, 1):
            print(f"{i}) {dof.name}= {dof.t:.2f}")
        print("7) Back\n")
        
        choice = get_valid_int_input("Choice: ", 1, 7)
        if choice == 7:
            return
            
        i = choice - 1
        node.dof[i].t = get_valid_float_input(f"Enter new target value for {node.dof[i].name}: ")

def setOffset(dof, sensorsData):
    i = 0
    duration = 1000
    while i < duration:
        dof[0].offset += sensorsData[0]
        dof[1].offset += sensorsData[1]
        dof[2].offset += sensorsData[2]
        dof[3].offset += sensorsData[3]
        # dof[4].offset += sensorsData[4]
        # dof[5].offset += sensorsData[5]
        i += 1

    dof[0].offset = dof[0].offset / duration
    dof[1].offset = dof[1].offset / duration
    dof[2].offset = dof[2].offset / duration
    dof[3].offset = dof[3].offset / duration
    # dof[4].offset = dof[4].offset / duration
    # dof[5].offset = dof[5].offset / duration

    print(f'Depth Offset: {dof[0].offset}\nRoll Offset: {dof[1].offset}\nPitch Offset: {dof[2].offset}\nYaw Offset: {dof[3].offset}\nX Offset: {dof[4].offset}\nY Offset: {dof[5].offset}\n')

def resetValues(dof):
    for i in dof:
        for j in range(0, 3):
            i.pid[j] = 0
        i.t = 0
        i.offset = 0



def transform_values(arr):
    """
    Transform control values to PWM values
    """
    result = []
    for x in arr:
        x = round(x, 2)
        if x <= 2 and x >= -2:
            if x > 0 and x < 0.2:
                x = 0.2
            elif x < 0 and x > -0.2:
                x = -0.2
            result.append(7.5 + x)
        else:
            raise ValueError("Each input must be between -2 and 2.")
    return result


def deg2rad_IBConv(rA, pA, wA, T_out, R_out):
    """
    Convert degrees to radians and apply coordinate transformation
    """
    rAd = rA * m.pi / 180
    pAd = pA * m.pi / 180
    wAd = wA * m.pi / 180

    DCMTranspose = np.array([
        [m.cos(pAd) * m.cos(wAd), m.cos(pAd) * m.sin(wAd), -m.sin(pAd)],
        [m.sin(rAd) * m.sin(pAd) * m.cos(wAd) - m.cos(rAd) * m.sin(wAd), m.sin(rAd) * m.sin(pAd) * m.sin(wAd) + m.cos(rAd) * m.cos(wAd), m.sin(rAd) * m.cos(pAd)],
        [m.cos(rAd) * m.sin(pAd) * m.cos(wAd) + m.sin(rAd) * m.sin(wAd), m.cos(rAd) * m.sin(pAd) * m.sin(wAd) - m.sin(rAd) * m.cos(wAd), m.cos(rAd) * m.cos(pAd)]
    ])
    B_Trans_out = np.matmul(DCMTranspose, T_out)
    B_comb_out = np.vstack((B_Trans_out, R_out))
    return B_comb_out

def save_to_csv(dof_list, filename="dof_data.csv"):
    """
    Save DOF configuration to CSV file
    """
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["DOF Name", "P", "I", "D", "Target", "Offset"])
        for dof in dof_list:
            writer.writerow([dof.name, dof.pid[0], dof.pid[1], dof.pid[2], dof.t, dof.offset])

def load_from_csv(filename="dof_data.csv"):
    """
    Load DOF configuration from CSV file
    """
    dof_list = []
    try:
        with open(filename, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header row
            for row in reader:
                name = row[0]
                pid = [float(row[1]), float(row[2]), float(row[3])]
                t = float(row[4])
                offset = float(row[5])
                dof = Dof(name, pid, t, offset)
                dof_list.append(dof)
    except Exception as e:
        print(f"Error reading CSV file: {e}. Initializing with default values.")

    # Ensure the list has exactly 6 DOFs
    default_dofs = [
        Dof("Depth"),
        Dof("Roll"),
        Dof("Pitch"),
        Dof("Yaw"),
        Dof("X"),
        Dof("Y")
    ]
    
    # If no DOFs were loaded or not enough, use defaults
    if len(dof_list) < 6:
        for i, default_dof in enumerate(default_dofs):
            if i < len(dof_list):
                dof_list[i].name = default_dof.name  # Preserve existing DOF data
            else:
                dof_list.append(default_dof)  # Add missing DOFs

    return dof_list


class Thrusters(Node):
    """
    Thrusters Node for AUV control
    Receives orientation deviations from Navigation and controls thrusters
    """

    def __init__(self):
        super().__init__('thrustersNode')
        
        #####################################################
        # SECTION: ROS PUBLISHERS AND SUBSCRIBERS
        #####################################################
        
        # Create Orientation Subscriber - receives deviations from Navigation
        self.orientation_subscription = self.create_subscription(
            Float32MultiArray,
            'orientation',
            self.listener_callback_orientation,
            10)
        self.orientation_subscription  
        
        # Create Thruster Output Publisher - for monitoring
        self.thruster_publisher = self.create_publisher(
            String, 
            'thruster_output', 
            10)
        
        # Create Sensors Subscriber
        self.sensors_subscription = self.create_subscription(
            Float32MultiArray,
            'sensors',
            self.listener_callback_sensors,
            10)
        self.sensors_subscription
        
        # Create User Input Subscriber - receives DOF targets from user
        self.userIn_subscription = self.create_subscription(
            Float32MultiArray,
            'userIn',
            self.listener_callback_userIn,
            10)
        self.userIn_subscription
        
        #####################################################
        # SECTION: STATE VARIABLES AND PARAMETERS
        #####################################################
        
        # Load DOF configuration
        self.dof = load_from_csv()
        
        # Sensor data storage
        self.sensors_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Control mode flags
        self.is_disabled = False
        self.is_hover_mode = False
        
        # Thruster configuration
        self.num_thrusters = 8
        self.thruster_values = [7.5] * self.num_thrusters  # Neutral values (7.5)
        
        # Thruster allocation matrix
        self.binary_alloc = np.array([
            [-0.5, 0, 0, 0.5, 0.5, 0, 0, -0.5],  # X
            [0.5, 0, 0, -0.5, 0.5, 0, 0, -0.5],  # Y
            [0, -1, -1, 0, 0, -1, -1, 0],        # Z
            [0, 1, -1, 0, 0, 1, -1, 0],          # Roll
            [0, -1, -1, 0, 0, 1, 1, 0],          # Pitch
            [-1, 0, 0, -1, 1, 0, 0, 1]           # Yaw
        ])
        
        # Compute inverse allocation matrix
        self.inv_binary_alloc = np.linalg.pinv(self.binary_alloc)
        
        # Serial communication for hardware control
        self.serial_port = None
        self.try_connect_serial()
        
        # Result string for thruster commands
        self.result = ",".join(["7.5"] * 8) + "\n"  # Initialize with neutral values
        
        # Logging
        self.get_logger().info("Thrusters node initialized")
        self.get_logger().info("Listening for orientation deviations on 'orientation' topic")
        self.get_logger().info("Listening for user input on 'userIn' topic")
    
    def try_connect_serial(self):
        """Try to connect to the thruster control board via serial"""
        try:
            # Modify port and baudrate according to your hardware
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',  # Change to your port
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info("Connected to thruster control board")
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to thruster control board: {str(e)}")
            self.get_logger().info("Running in simulation mode (no hardware control)")
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        # Save DOF configuration
        save_to_csv(self.dof)
        
        # Set thrusters to neutral before shutting down
        self.set_neutral_thrusters()
        
        # Close serial port if open
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed")
    
    #####################################################
    # SECTION: SUBSCRIBER CALLBACKS
    #####################################################
    
    def listener_callback_orientation(self, msg):
        """
        Process orientation deviations from Navigation
        Format: [roll_dev, pitch_dev, yaw_dev, surge, sway]
        """
        if self.is_disabled:
            return
            
        if len(msg.data) >= 5:
            roll_dev = float(msg.data[0])
            pitch_dev = float(msg.data[1])
            yaw_dev = float(msg.data[2])
            surge = float(msg.data[3])
            sway = float(msg.data[4])
            
            self.get_logger().debug(
                f'Received deviations - Roll: {roll_dev:.2f}, ' +
                f'Pitch: {pitch_dev:.2f}, ' +
                f'Yaw: {yaw_dev:.2f}, ' +
                f'Surge: {surge:.2f}, ' +
                f'Sway: {sway:.2f}'
            )
            
            # Update DOF targets based on deviations
            # Note: We're using the deviations to adjust the targets relative to current values
            self.dof[1].t = self.dof[1].a + roll_dev   # Roll
            self.dof[2].t = self.dof[2].a + pitch_dev  # Pitch
            self.dof[3].t = self.dof[3].a + yaw_dev    # Yaw
            
            # For X (surge) we use the direct value
            if abs(surge) > 0.1:  # Only update if significant
                self.dof[4].t = surge
            
            # For Y (sway) we use the direct value
            if abs(sway) > 0.1:  # Only update if significant
                self.dof[5].t = sway
            
            # Process updated targets and send to thrusters
            self.process_control()
        else:
            self.get_logger().warn("Received malformed orientation message")
    
    def listener_callback_sensors(self, msg):
        """Process sensor data (orientation, depth)"""
        if len(msg.data) > 0:
            # Store sensor data
            self.sensors_data = list(msg.data)
            
            # Update actual values for each DOF
            for i in range(min(len(self.dof), len(self.sensors_data))):
                self.dof[i].a = self.sensors_data[i] - self.dof[i].offset
            
            self.get_logger().debug(
                f'Sensor data - Depth: {self.dof[0].a:.2f}, ' +
                f'Roll: {self.dof[1].a:.2f}, ' +
                f'Pitch: {self.dof[2].a:.2f}, ' +
                f'Yaw: {self.dof[3].a:.2f}'
            )
    
    def listener_callback_userIn(self, msg):
        """
        Process user input commands
        Format: [depth, roll, pitch, yaw, x, y]
        """
        if len(msg.data) == 6:
            # Update DOF targets based on user input
            self.dof[0].t += float(msg.data[0])  # Depth
            self.dof[1].t += float(msg.data[1])   # Roll
            self.dof[2].t += float(msg.data[2])   # Pitch
            self.dof[3].t += float(msg.data[3])   # Yaw
            self.dof[4].t += float(msg.data[4])   # X
            self.dof[5].t += float(msg.data[5])   # Y
            
            self.get_logger().debug(
                f'Received user input - Depth: {self.dof[0].t:.2f}, ' +
                f'Roll: {self.dof[1].t:.2f}, ' +
                f'Pitch: {self.dof[2].t:.2f}, ' +
                f'Yaw: {self.dof[3].t:.2f}, ' +
                f'X: {self.dof[4].t:.2f}, ' +
                f'Y: {self.dof[5].t:.2f}'
            )
            
            # Process updated targets and send to thrusters
            self.process_control()
        else:
            self.get_logger().warn("Received malformed user input message")
    
    #####################################################
    # SECTION: THRUSTER CONTROL
    #####################################################
    
    def process_control(self):
        """
        Process control using PID controllers and send to thrusters
        """
        # PID Controller setup
        x_pid = PID.Pidcontroller(
            self.dof[4].pid[0],
            self.dof[4].pid[1],
            self.dof[4].pid[2],
            setpoint=self.dof[4].t
        )
        y_pid = PID.Pidcontroller(
            self.dof[5].pid[0],
            self.dof[5].pid[1],
            self.dof[5].pid[2],
            setpoint=self.dof[5].t
        )
        z_pid = PID.Pidcontroller(
            self.dof[0].pid[0],
            self.dof[0].pid[1],
            self.dof[0].pid[2],
            setpoint=self.dof[0].t
        )
        r_pid = PID.Pidcontroller(
            self.dof[1].pid[0],
            self.dof[1].pid[1],
            self.dof[1].pid[2],
            setpoint=self.dof[1].t
        )
        p_pid = PID.Pidcontroller(
            self.dof[2].pid[0],
            self.dof[2].pid[1],
            self.dof[2].pid[2],
            setpoint=self.dof[2].t
        )
        w_pid = PID.Pidcontroller(
            self.dof[3].pid[0],
            self.dof[3].pid[1],
            self.dof[3].pid[2],
            setpoint=self.dof[3].t
        )

        # Calculate PID outputs
        x_out = x_pid(self.dof[4].a)
        y_out = y_pid(self.dof[5].a)
        z_out = z_pid(self.dof[0].a)
        r_out = r_pid(self.dof[1].a)
        p_out = p_pid(self.dof[2].a)
        w_out = w_pid(self.dof[3].a)

        # Create control vectors
        Trans_ctrl_out = np.array([[x_out], [y_out], [z_out]])
        Rot_ctrl_out = np.array([[r_out], [p_out], [w_out]])

        # Apply coordinate transformation
        B_comb_out = deg2rad_IBConv(
            self.dof[1].a,
            self.dof[2].a,
            self.dof[3].a,
            Trans_ctrl_out,
            Rot_ctrl_out
        )

        # Calculate thruster outputs
        final_out = np.matmul(self.inv_binary_alloc, B_comb_out)
        clipped = np.clip(final_out, -2, 2)
        
        # Apply forward motion if needed
        if self.is_hover_mode:
            forwards = 0  # No forward motion in hover mode
        else:
            # Forward motion based on X target
            forwards = self.dof[4].t if abs(self.dof[4].t) > 0.1 else 0
        
        # Apply translational motion
        new_clipped = trans_motion(forwards, 0, clipped)  # No lateral motion (y=0)
        final_clipped = np.clip(new_clipped, -2, 2)
        
        # Convert to PWM values
        flat = final_clipped.flatten()
        pwm = transform_values(flat)
        pwmr = [round(x, 2) for x in pwm]
        
        # Format as string and send
        str_values = [str(value) for value in pwmr]
        self.result = ",".join(str_values) + "\n"
        
        # Send to thrusters
        self.send_thruster_commands()
    
    def set_neutral_thrusters(self):
        """Set all thrusters to neutral position"""
        self.thruster_values = [7.5] * self.num_thrusters
        self.result = ",".join([str(val) for val in self.thruster_values]) + "\n"
        self.send_thruster_commands()
    
    def send_thruster_commands(self):
        """Send commands to thrusters via serial and publish for monitoring"""
        # Publish thruster values for monitoring
        msg = String()
        msg.data = self.result.strip()
        self.thruster_publisher.publish(msg)
        
        # Send to hardware if connected
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(self.result.encode('utf-8'))
                self.get_logger().debug(f"Sent to thrusters: {self.result.strip()}")
            except Exception as e:
                self.get_logger().error(f"Failed to send thruster commands: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    thrustersNode = Thrusters()

    rclpy.spin(thrustersNode)

    thrustersNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()