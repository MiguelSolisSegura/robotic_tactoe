#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            Image,
            '/demo_cam/camera1/image_raw', # SIMULATION
            self.image_callback,
            10,
            callback_group=self.callback_group)
        
        self.publisher_ = self.create_publisher(Image, 'image_processed', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        try:
            # Convert the ROS image message to a BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Define trapezoidal ROI points
        pts = np.array([[675, 325], [1235, 325], [1380, 820], [520, 820]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        
        mask = np.zeros(gray_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        
        # Apply the mask to the image
        masked_image = cv2.bitwise_and(gray_image, gray_image, mask=mask)
        
        # Fill everything outside the ROI with black
        masked_image[mask == 0] = 0

        segmented_image, num_labels, labels_im = self.image_segmentation(masked_image)
        
        processed_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding='bgr8')
        
        self.publisher_.publish(processed_image_msg)
        self.get_logger().info('Published processed image')

    def image_segmentation(self, img, dilate_iterations=1):
        # Ensure the input image is a numpy array
        if not isinstance(img, np.ndarray):
            raise ValueError("Input image must be a numpy array")

        # Apply binary threshold
        _, binary_img = cv2.threshold(img, 95, 255, cv2.THRESH_BINARY)

        # Apply dilation
        kernel = np.ones((3, 3), np.uint8)
        dilated_img = cv2.dilate(binary_img, kernel, iterations=dilate_iterations)

        # Label connected components
        num_labels, labels_im = cv2.connectedComponents(dilated_img)

        # Create an output image with colored segments
        output_image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Find the largest component
        largest_component = 1 + np.argmax(np.bincount(labels_im.flat)[1:])

        # Color the components
        cross_n = 0
        circle_n = 0
        for label in range(1, num_labels):
            if label == largest_component:
                output_image[labels_im == label] = [255, 255, 0]  # Cyan color for the largest component (grid)
            else:
                # Extract the component
                component_mask = (labels_im == label).astype(np.uint8) * 255
                # Calculate geometric middle
                x, y, w, h = cv2.boundingRect(component_mask)
                middle_y = y + h // 2
                start_x = x + w // 3
                end_x = x + 2 * w // 3
                middle_line = component_mask[middle_y:middle_y + 1, start_x:end_x]

                # Check for white pixels in the middle third
                if np.count_nonzero(middle_line) == 0:
                    output_image[labels_im == label] = [0, 255, 0]  # Green color for circles
                    circle_n += 1
                else:
                    output_image[labels_im == label] = [0, 0, 255]  # Red color for crosses
                    cross_n += 1

        # Add text annotations
        board_present = num_labels > 1
        board_text = f"Board: {int(board_present)}"
        cross_text = f"Cross: {cross_n}"
        circle_text = f"Circle: {circle_n}"
        cv2.putText(output_image, board_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(output_image, cross_text, (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(output_image, circle_text, (10, 260), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

        return output_image, num_labels, labels_im

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
