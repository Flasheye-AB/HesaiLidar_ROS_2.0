#!/usr/bin/env python3
"""
ROS2 Range Image Viewer (Live OpenCV) for Hesai OT128 with RemakeConfig

Fast live viewer using OpenCV for real-time display.
Shows range image as grayscale with interactive zoom.

Controls:
    Left/Right Arrow: Pan zoom window
    S: Save current frame
    Q or ESC: Exit

Usage:
    python3 ros2_range_image_viewer_live.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import os
from datetime import datetime

class RangeImageViewerLive(Node):
    def __init__(self):
        super().__init__('range_image_viewer_live')

        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.pointcloud_callback,
            10)

        self.frame_count = 0
        self.zoom_center_ratio = 0.5  # Center of zoom window as ratio (0.5 = middle)
        self.current_display_img = None  # Store current image for saving

        # Setup save directory
        self.save_dir = 'live_snapshots'
        os.makedirs(self.save_dir, exist_ok=True)

        # Create OpenCV window (will display at half scale)
        # Full image: 3600x128, Zoom: 3600x640, Total with borders: 3632x800
        cv2.namedWindow('Hesai OT128 Range Image (Live)', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Hesai OT128 Range Image (Live)', 3632 // 2, 800 // 2)

        self.get_logger().info('Range Image Viewer (Live) started')
        self.get_logger().info('Controls: Left/Right arrows to pan, S to save, Q/ESC to exit')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud and display range image"""
        self.frame_count += 1

        # Check if organized
        if msg.height == 1:
            self.get_logger().warn('Point cloud is unorganized (height=1)!')
            return

        height = msg.height  
        width = msg.width    

        print(f"Received PointCloud2: height={height}, width={width}")

        # Extract point cloud data
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=False):
            points.append(point)

        points = np.array(points, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])

        # Reshape to grid
        grid = points.reshape(height, width)

        # Calculate distance
        distances = np.sqrt(grid['x']**2 + grid['y']**2 + grid['z']**2)

        range_img = distances

        # Create color image (BGR)
        display_img = self.create_range_display(range_img)
        self.current_display_img = display_img  # Store full-size for saving

        # Create half-scale version for display
        display_h, display_w = display_img.shape[:2]
        if display_w > 1800:
            display_img_half = cv2.resize(display_img, (display_w // 2, display_h // 2), interpolation=cv2.INTER_AREA)
        else:
            display_img_half = display_img

        # Display half-scale
        cv2.imshow('Hesai OT128 Range Image (Live)', display_img_half)

        # Check for keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # 'q' or ESC
            self.get_logger().info('Exit requested')
            rclpy.shutdown()
        elif key == ord('s'):  # 's' to save
            self.save_current_image()
        elif key == 81 or key == 2 or key==ord('a'):  # Left arrow (81=Linux, 2=some systems)
            self.zoom_center_ratio = max(0.1, self.zoom_center_ratio - 0.05)
        elif key == 83 or key == 3 or key==ord('d'):  # Right arrow (83=Linux, 3=some systems)
            self.zoom_center_ratio = min(0.9, self.zoom_center_ratio + 0.05)

    def create_range_display(self, range_img):
        """Create full display with overview and zoom"""
        # range_img shape: (rings, azimuth) - could be (128, 3600) or other dimensions
        img_height, img_width = range_img.shape  # Actual image dimensions

        # Normalize and create colorized image
        valid_mask = range_img > 0
        colored_full = np.zeros((img_height, img_width, 3), dtype=np.uint8)
        print(f"Range image shape: {range_img.shape}, valid points: {np.sum(valid_mask)}")

        if np.any(valid_mask):
            valid_distances = range_img[valid_mask]
            vmin, vmax = np.percentile(valid_distances, [1, 99])

            # Normalize valid distances
            normalized = np.zeros_like(range_img, dtype=np.float32)
            normalized[valid_mask] = np.clip((range_img[valid_mask] - vmin) / (vmax - vmin), 0, 1)

            # Convert to grayscale (inverted: close=bright, far=dark)
            gray_values = (255 * (1.0 - normalized)).astype(np.uint8)

            # Create BGR image: valid points = grayscale, invalid = red
            colored_full[:, :, 0] = np.where(valid_mask, gray_values, 0)    # B
            colored_full[:, :, 1] = np.where(valid_mask, gray_values, 0)    # G
            colored_full[:, :, 2] = np.where(valid_mask, gray_values, 128)  # R (red for invalid)
        else:
            colored_full[:, :, 2] = 128  # All red if no data

        # Overview: Keep original dimensions (no resize)
        overview = colored_full.copy()

        # Add 16 pixel blue border to overview
        overview = cv2.copyMakeBorder(overview, 16, 16, 16, 16, cv2.BORDER_CONSTANT, value=(255, 0, 0))  # Blue border

        # Skip zoom for narrow images (AT128 has width=1260, OT128 has width=3600)
        if img_width <= 1800:
            # No zoom needed for narrow FOV sensors like AT128
            return overview

        # Calculate zoom region boundaries (center 20% of azimuth)
        zoom_width_pixels = int(img_width * 0.20)  # 20% of azimuth
        zoom_center = int(img_width * self.zoom_center_ratio)
        zoom_start = max(0, zoom_center - zoom_width_pixels // 2)
        zoom_end = min(img_width, zoom_center + zoom_width_pixels // 2)

        # Draw green rectangle on overview showing zoom region (offset by border)
        rect_x1 = zoom_start + 16
        rect_x2 = zoom_end + 16
        cv2.rectangle(overview, (rect_x1, 16), (rect_x2, img_height - 1 + 16), (0, 255, 0), 1)

        # Extract and zoom region (5x zoom of center 20%)
        zoom_region = colored_full[:, zoom_start:zoom_end]
        zoomed_width = (zoom_end - zoom_start) * 5
        zoomed_height = img_height * 5
        zoomed = cv2.resize(zoom_region, (zoomed_width, zoomed_height), interpolation=cv2.INTER_NEAREST)

        # Add 16 pixel blue border to zoomed
        zoomed = cv2.copyMakeBorder(zoomed, 16, 16, 16, 16, cv2.BORDER_CONSTANT, value=(255, 0, 0))  # Blue border

        # Add sparse ring boundaries to zoom (if img_height is around 128)
        if img_height >= 24 and img_height <= 256:
            ring24_y_zoom = 24 * 5 + 16
            ring87_y_zoom = 87 * 5 + 16
            # cv2.line(zoomed, (16, ring24_y_zoom), (zoomed_width + 16, ring24_y_zoom), (0, 255, 255), 2)  # Cyan
            # cv2.line(zoomed, (16, ring87_y_zoom), (zoomed_width + 16, ring87_y_zoom), (0, 255, 255), 2)  # Cyan

        # Stack vertically
        combined = np.vstack([overview, zoomed])

        return combined

    def save_current_image(self):
        """Save the current display image to disk"""
        if self.current_display_img is None:
            self.get_logger().warn('No image to save yet')
            return

        # Generate filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # milliseconds
        filename = f'range_image_live_{timestamp}_frame_{self.frame_count:04d}.png'
        filepath = os.path.join(self.save_dir, filename)

        # Save image (PNG is lossless by default, but set compression to max quality)
        cv2.imwrite(filepath, self.current_display_img, [cv2.IMWRITE_PNG_COMPRESSION, 0])

        print(f'Saved: {filepath}')
        self.get_logger().info(f'Saved snapshot to: {filename}')

def main(args=None):
    rclpy.init(args=args)
    viewer = RangeImageViewerLive()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.get_logger().info(f'Total frames processed: {viewer.frame_count}')
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
