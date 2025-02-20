#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')
        
        # Declare parameters with default values
        self.declare_parameter('hud', True)
        self.declare_parameter('crosshair', True)
        self.declare_parameter('show_temp', True)
        self.declare_parameter('device', 0)
        self.declare_parameter('scale', 3)
        self.declare_parameter('alpha', 1.0)  # Contrast
        self.declare_parameter('colormap', 0)
        self.declare_parameter('blur_radius', 0)
        self.declare_parameter('threshold', 2)
        # New orientation parameter (allowed values: 0, 90, 180, 270)
        self.declare_parameter('orientation', 0)

        # Get parameter values
        self.hud = self.get_parameter('hud').value
        self.crosshair = self.get_parameter('crosshair').value
        self.show_temp = self.get_parameter('show_temp').value
        self.device = self.get_parameter('device').value
        self.scale = self.get_parameter('scale').value
        self.alpha = self.get_parameter('alpha').value
        self.colormap = self.get_parameter('colormap').value
        self.rad = self.get_parameter('blur_radius').value
        self.threshold = self.get_parameter('threshold').value
        self.orientation = self.get_parameter('orientation').value

        # ROS Publishers
        self.raw_pub = self.create_publisher(Image, 'thermal_image/raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'thermal_image/compressed', 10)
        self.bridge = CvBridge()
        
        # Camera initialization
        self.cap = cv2.VideoCapture(f'/dev/video{self.device}', cv2.CAP_V4L2)
        
        # Raspberry Pi detection
        self.is_pi = self.check_raspberrypi()
        if self.is_pi:
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
        else:
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, False)

        # Camera parameters
        self.width = 256
        self.height = 192
        self.new_width = self.width * self.scale
        self.new_height = self.height * self.scale

        # Create timer for camera processing
        self.timer = self.create_timer(0.1, self.process_frame)

    def check_raspberrypi(self):
        try:
            with open('/sys/firmware/devicetree/base/model', 'r') as m:
                return 'raspberry pi' in m.read().lower()
        except Exception:
            return False

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame")
            return

        # Split frame into image and thermal data
        imdata, thdata = np.array_split(frame, 2)

        # Temperature calculations
        hi = thdata[96][128][0]
        lo = thdata[96][128][1] * 256
        raw_temp = hi + lo
        temp = round((raw_temp / 64) - 273.15, 2)

        # Process image: convert and scale
        bgr = cv2.cvtColor(imdata, cv2.COLOR_YUV2BGR_YUYV)
        bgr = cv2.convertScaleAbs(bgr, alpha=self.alpha)
        bgr = cv2.resize(bgr, (self.new_width, self.new_height), interpolation=cv2.INTER_CUBIC)
        
        if self.rad > 0:
            bgr = cv2.blur(bgr, (self.rad, self.rad))

        # Apply colormap to get a thermal heatmap
        heatmap = self.apply_colormap(bgr)

        # Rotate the heatmap using cv2.rotate for 90° increments
        if self.orientation == 90:
            rotated = cv2.rotate(heatmap, cv2.ROTATE_90_CLOCKWISE)
        elif self.orientation == 180:
            rotated = cv2.rotate(heatmap, cv2.ROTATE_180)
        elif self.orientation == 270:
            rotated = cv2.rotate(heatmap, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            rotated = heatmap

        # Get final image dimensions from the rotated image
        final_height, final_width = rotated.shape[:2]
        center_x, center_y = final_width // 2, final_height // 2

        # Draw crosshair at the center of the final (rotated) image
        if self.crosshair:
            # Draw white lines for high contrast crosshair
            cv2.line(rotated, (center_x, center_y - 20), (center_x, center_y + 20), (255,255,255), 2)
            cv2.line(rotated, (center_x - 20, center_y), (center_x + 20, center_y), (255,255,255), 2)
        
        # Draw temperature text above the crosshair (fixed offset)
        if self.show_temp:
            temp_text = f'{temp}C'
            # You can adjust the offsets as needed
            text_offset_x = 10
            text_offset_y = 30
            cv2.putText(rotated, temp_text, (center_x + text_offset_x, center_y - text_offset_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        # Draw HUD at the top-left corner of the rotated image
        if self.hud:
            hud_bg_width = 160
            hud_bg_height = 120
            # Draw a filled rectangle for the HUD background
            cv2.rectangle(rotated, (0, 0), (hud_bg_width, hud_bg_height), (0,0,0), -1)
            texts = [
                f"Scale: {self.scale}",
                f"Contrast: {self.alpha:.1f}",
                f"Blur: {self.rad}",
                f"Threshold: {self.threshold}C"
            ]
            for i, text in enumerate(texts):
                y = 20 + i * 30
                cv2.putText(rotated, text, (10, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        # Publish images
        self.publish_images(rotated)

    def apply_colormap(self, bgr):
        colormaps = [
            cv2.COLORMAP_JET, cv2.COLORMAP_HOT, cv2.COLORMAP_MAGMA,
            cv2.COLORMAP_INFERNO, cv2.COLORMAP_PLASMA, cv2.COLORMAP_BONE,
            cv2.COLORMAP_SPRING, cv2.COLORMAP_AUTUMN, cv2.COLORMAP_VIRIDIS,
            cv2.COLORMAP_PARULA, cv2.COLORMAP_RAINBOW
        ]
        heatmap = cv2.applyColorMap(bgr, colormaps[self.colormap])
        if self.colormap == 10:  # Rainbow case
            heatmap = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
        return heatmap

    def publish_images(self, final_img):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(final_img, "bgr8")
            self.raw_pub.publish(ros_image)
            
            _, jpeg = cv2.imencode('.jpg', final_img)
            comp_msg = CompressedImage()
            comp_msg.format = "jpeg"
            comp_msg.data = jpeg.tobytes()
            self.compressed_pub.publish(comp_msg)
        except Exception as e:
            self.get_logger().error(f"Publishing error: {str(e)}")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
