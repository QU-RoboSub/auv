import rclpy
from rclpy.node import Node
import numpy as np
import math as m
import serial
import time
import csv
from pathlib import Path

from std_msgs.msg import String, Float32MultiArray
from . import pid as PID

# Constants
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