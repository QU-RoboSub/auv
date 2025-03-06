import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import depthai as dai
import cv2
import numpy as np
import time
import threading
import queue
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from scripts.euler_from_quat import euler_from_quat
import math
import subprocess

def quaternion_to_euler(w, x, y, z):
    rotation_object = R.from_quat([x, y, z, w])
    euler_angles = euler_from_quat(rotation_object, 'yxz', degrees=False)
    pitch, roll, yaw = euler_angles
    return roll, pitch, yaw

class FrontCamera(Node):

    def __init__(self):
        super().__init__('frontCameraNode')
        
        # ROS2 Publishers
        self.publisher_sensors = self.create_publisher(Float32MultiArray, 'sensors', 10)
        self.publisher_yolo = self.create_publisher(Float32MultiArray, 'frontYolo', 10)
        self.publisher_feed = self.create_publisher(Float32MultiArray, 'frontFeed', 10)
        
        # Data queues for thread-safe communication
        self.data_queue_sensors = queue.Queue()
        self.data_queue_yolo = queue.Queue()
        self.data_queue_feed = queue.Queue()
        
        # Timers to process queues and publish data
        self.create_timer(0.001, self.timer_callback_sensors)  # High frequency for sensors
        self.create_timer(0.001, self.timer_callback_yolo)     # High frequency for YOLO
        self.create_timer(0.001, self.timer_callback_feed)     # High frequency for video feed
        
        # DepthAI setup
        self.setup_depthai()
        
        # Thread for DepthAI processing
        self.running = True
        self.processing_thread = threading.Thread(target=self.process_depthai_data)
        self.processing_thread.start()

        # Error handling variables
        self.error_count = 0
        self.MAX_ERRORS = 5  # Maximum number of consecutive errors before restart
        self.ERROR_TIMEOUT = 2.0  # Seconds without successful read before considering an error

    def setup_depthai(self):
        try:
            # Configuration
            model_path = '/home/quauv/prequalAuv/src/sensors/sensors/models/yolov8n_coco_640x352.blob'

            self.label_map = ["person", "car", "dog"]
            NN_INPUT_SIZE = (640, 352)
            DISPLAY_SIZE = (640, 640)

            # Pipeline setup
            pipeline = dai.Pipeline()

            # Camera configuration
            cam = pipeline.create(dai.node.ColorCamera)
            cam.setPreviewSize(DISPLAY_SIZE[0], DISPLAY_SIZE[1])
            cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
            cam.setFps(30)
            cam.setInterleaved(False)
            cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

            # Image manip
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(NN_INPUT_SIZE[0], NN_INPUT_SIZE[1])
            manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
            cam.preview.link(manip.inputImage)

            # YOLO network
            nn = pipeline.create(dai.node.YoloDetectionNetwork)
            nn.setBlobPath(model_path)
            nn.setConfidenceThreshold(0.5)
            nn.setNumClasses(len(self.label_map))
            nn.setCoordinateSize(4)
            nn.input.setBlocking(False)
            manip.out.link(nn.input)

            # IMU configuration
            imu = pipeline.create(dai.node.IMU)
            imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
            imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 500)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            # Create outputs
            xout_nn = pipeline.create(dai.node.XLinkOut)
            xout_nn.setStreamName("detections")
            xout_imu = pipeline.create(dai.node.XLinkOut)
            xout_imu.setStreamName("imu")
            xout_video = pipeline.create(dai.node.XLinkOut)
            xout_video.setStreamName("video")

            # Link nodes
            nn.out.link(xout_nn.input)
            imu.out.link(xout_imu.input)
            cam.preview.link(xout_video.input)

            # Device connection
            self.device = dai.Device(pipeline)
            self.q_nn = self.device.getOutputQueue("detections", maxSize=1, blocking=False)
            self.q_imu = self.device.getOutputQueue("imu", maxSize=50, blocking=False)
            self.q_video = self.device.getOutputQueue("video", maxSize=1, blocking=False)

            # Video writer
            self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Use XVID codec for AVI
            self.video_filename = f'recording_{datetime.now().strftime("%Y%m%d_%H%M%S")}.avi'
            self.out = None

            self.error_count = 0  # Reset error count on successful setup
            self.get_logger().info("DepthAI setup completed successfully")

        except Exception as e:
            self.get_logger().error(f"Error during DepthAI setup: {str(e)}")
            self.error_count += 1
            if self.error_count >= self.MAX_ERRORS:
                self.get_logger().error("Too many consecutive errors during setup, attempting recovery")
                self.recover()

    def frame_norm(self, frame, bbox):
        return (np.clip(np.array(bbox), 0, 1) * np.array([*frame.shape[:2], *frame.shape[:2]])[::-1]).astype(int)

    def process_depthai_data(self):
        last_imu_print = time.time()
        while self.running:
            try:
                # Process video frame
                video_frame = self.q_video.tryGet()
                if video_frame is not None:
                    frame = video_frame.getCvFrame()
                    
                    # Process detections
                    detections = self.q_nn.tryGet()
                    if detections is not None:
                        yolo_msg = Float32MultiArray()
                        yolo_data = []
                        for detection in detections.detections:
                            bbox = self.frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                            label = self.label_map[detection.label]
                            # Convert any non-float data to float
                            confidence = float(detection.confidence)
                            x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
                            class_id = float(detection.label)  # Use numerical ID instead of string label
                            yolo_data.extend([confidence, x1, y1, x2, y2, class_id])
                            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                            cv2.putText(frame, f"{label}:{detection.confidence:.2f}", (bbox[0] + 10, bbox[1] + 20), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        yolo_msg.data = yolo_data
                        self.data_queue_yolo.put(yolo_msg)
                    
                    # Process video feed
                    feed_msg = Float32MultiArray()
                    feed_msg.data = [0.0] * 6  # Placeholder data
                    self.data_queue_feed.put(feed_msg)
                    
                    # Write to video file
                    if self.out is None:
                        h, w = frame.shape[:2]
                        self.out = cv2.VideoWriter(self.video_filename, self.fourcc, 30, (w, h))
                        if not self.out.isOpened():
                            self.get_logger().error("Failed to open video writer")
                    self.out.write(frame)
                    cv2.imshow("Preview", frame)
                    cv2.waitKey(1)

                # Process IMU data
                current_time = time.time()
                if current_time - last_imu_print > 0.1:
                    imu_data = self.q_imu.tryGetAll()
                    if imu_data:
                        # Get only the last packet with rotation data
                        latest_rot = None
                        for packet in reversed(imu_data):
                            for report in reversed(packet.packets):
                                if hasattr(report, 'rotationVector'):
                                    latest_rot = report.rotationVector
                                    break
                            if latest_rot:
                                break
                        
                        if latest_rot:
                            try:
                                roll, pitch, yaw = quaternion_to_euler(
                                    latest_rot.real, latest_rot.i, latest_rot.j, latest_rot.k)
                                
                                # Explicit validation and conversion to float
                                roll = float(roll)
                                pitch = float(pitch)
                                yaw = float(yaw)
                                
                                if (not math.isnan(roll) and not math.isinf(roll) and
                                    not math.isnan(pitch) and not math.isinf(pitch) and
                                    not math.isnan(yaw) and not math.isinf(yaw)):
                                    
                                    sensor_msg = Float32MultiArray()
                                    sensor_msg.data = [roll, pitch, yaw]
                                    self.data_queue_sensors.put(sensor_msg)
                                    
                                else:
                                    self.get_logger().warn("Invalid IMU values detected, skipping")
                            except Exception as e:
                                self.get_logger().warn(f"Error processing IMU data: {e}")
                        
                        last_imu_print = current_time

                # Exit on ESC
                if cv2.waitKey(1) == 27:
                    self.running = False

                self.error_count = 0  # Reset error count on successful processing

            except Exception as e:
                self.get_logger().error(f"Error during DepthAI processing: {str(e)}")
                self.error_count += 1
                
                # Close video file in case of error
                if self.out is not None:
                    self.out.release()
                    self.out = None
                    
                if self.error_count >= self.MAX_ERRORS:
                    self.get_logger().error(f"Too many consecutive errors ({self.error_count}), attempting recovery")
                    self.recover()
                    # Reset error count after recovery attempt
                    self.error_count = 0
                    time.sleep(2)  # Give time for recovery to complete

    def recover(self):
        """Attempt to recover from errors by reinitializing components."""
        self.get_logger().info("Attempting to recover from errors...")
        try:
            self.cleanup()
            time.sleep(2)  # Give more time for cleanup
            self.setup_depthai()
            self.get_logger().info("Recovery completed successfully")
        except Exception as e:
            self.get_logger().error(f"Recovery failed: {e}")
            # If recovery fails, we need to retry or potentially exit
            self.get_logger().error("Recovery failed, will retry on next error")

    def cleanup(self):
        """Clean up resources."""
        self.get_logger().info("Cleaning up resources...")
        if self.device is not None:
            self.device.close()
            self.device = None
        if self.out is not None:
            self.out.release()
            self.out = None
        cv2.destroyAllWindows()

    def timer_callback_sensors(self):
        while not self.data_queue_sensors.empty():
            msg = self.data_queue_sensors.get()
            self.publisher_sensors.publish(msg)
            
            # Check if msg.data has valid elements
            if len(msg.data) >= 3:
                # Get the roll, pitch, and yaw values
                roll = msg.data[0]
                pitch = msg.data[1]
                yaw = msg.data[2]
                self.get_logger().info('Publishing sensors: Roll: %.2f, Pitch: %.2f, Yaw: %.2f' % (roll, pitch, yaw))
            else:
                self.get_logger().warn('Received sensor data with unexpected length: %d' % len(msg.data))

    def timer_callback_yolo(self):
        while not self.data_queue_yolo.empty():
            msg = self.data_queue_yolo.get()
            self.publisher_yolo.publish(msg)
            self.get_logger().info('Publishing YOLO: %s' % ', '.join(['Confidence: %.2f, BBox: (%d, %d, %d, %d), Label: %s' % (msg.data[i], msg.data[i+1], msg.data[i+2], msg.data[i+3], msg.data[i+4], msg.data[i+5]) for i in range(0, len(msg.data), 6)]))

    def timer_callback_feed(self):
        while not self.data_queue_feed.empty():
            msg = self.data_queue_feed.get()
            self.publisher_feed.publish(msg)
            self.get_logger().info('Publishing feed: Placeholder data')

    def destroy_node(self):
        self.running = False
        self.processing_thread.join()
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    front_camera_node = FrontCamera()
    rclpy.spin(front_camera_node)
    front_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()