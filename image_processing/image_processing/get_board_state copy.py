#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger

from moveit_planning.srv import GetBoardState

def image_segmentation(img: np.ndarray, invert=False, roi=np.array([]), dilate_iterations=1) -> tuple:
    """
    Obtain a segmentation of the elements in the board.

    Parameters:
    - invert: Use `True` if the background of your board is white.
    - roi: Define it as a polygonal mask.

    Returns:
    - img: Processed image.
    - num_labels: Number of labels found.
    - labels_im: Labeled image with connected components.
    """
    if roi.size > 0:
        # Apply the mask to the image
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [roi], 255)
        img = cv2.bitwise_and(img, img, mask=mask)

    # Apply binary threshold
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    # Invert the binary image
    if invert:
        img = cv2.bitwise_not(img)

    # Apply dilation
    kernel = np.ones((3, 3), np.uint8)
    img = cv2.dilate(img, kernel, iterations=dilate_iterations)

    # Label connected components
    num_labels, labels_im = cv2.connectedComponents(img)

    return img, num_labels, labels_im

def find_largest_component(labels, num_labels):
    max_area = 0
    largest_label = 1
    for label in range(1, num_labels):
        area = np.sum(labels == label)
        if area > max_area:
            max_area = area
            largest_label = label
    return largest_label

def is_circle(component):
    x, y, w, h = cv2.boundingRect(component)
    mid_x = x + w // 2
    start_y = y + h // 3
    end_y = y + 2 * h // 3

    for i in range(start_y, end_y):
        if component[i, mid_x] == 255:
            return False

    return True

def get_board_state(labels, num_labels, largest_label, pts, segmented_image):
    board_size = 3
    board_state = np.zeros((board_size, board_size), dtype=int)
    
    cell_width = (max(pts[:, 0, 0]) - min(pts[:, 0, 0])) // board_size
    cell_height = (max(pts[:, 0, 1]) - min(pts[:, 0, 1])) // board_size
    
    for label in range(1, num_labels):
        if label == largest_label:
            continue
        component_mask = (labels == label).astype(np.uint8) * 255
        x, y, w, h = cv2.boundingRect(component_mask)
        
        # Determine cell indices
        cell_x = (x + w // 2 - min(pts[:, 0, 0])) // cell_width
        cell_y = (y + h // 2 - min(pts[:, 0, 1])) // cell_height
        
        # Ensure indices are within bounds
        if 0 <= cell_x < board_size and 0 <= cell_y < board_size:
            component = cv2.bitwise_and(segmented_image, segmented_image, mask=component_mask)
            if is_circle(component):
                board_state[cell_y, cell_x] = 2  # 'O'
            else:
                board_state[cell_y, cell_x] = 1  # 'X'
    
    return board_state

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
        
        self.get_board_state_service = self.create_service(
            GetBoardState,
            'get_board_state',
            self.get_board_state_callback,
            callback_group=self.callback_group)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_labels = None
        self.latest_num_labels = None
        self.latest_largest_label = None
        self.latest_pts = None
        self.latest_segmented_image = None

    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        #pts = np.array([[143, 62], [274, 62], [317, 192], [78, 192]], np.int32) # REAL
        pts = np.array([[675, 325], [1235, 325], [1380, 820], [520, 820]], np.int32) # SIMULATION
        pts = pts.reshape((-1, 1, 2))
        
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Call the image_segmentation function with the trapezoid ROI
        segmented_image, num_labels, labels_im = image_segmentation(gray_image, invert=False, roi=pts, dilate_iterations=0)
        segmented_image_colored = cv2.cvtColor(segmented_image, cv2.COLOR_GRAY2BGR)
        
        # Find the largest component (the grid)
        largest_label = find_largest_component(labels_im, num_labels)
        
        # Change the pixels of the grid to blue color
        segmented_image_colored[labels_im == largest_label] = [255, 255, 0]
        
        # Process other components
        for label in range(1, num_labels):
            if label == largest_label:
                continue
            component_mask = (labels_im == label).astype(np.uint8) * 255
            component = cv2.bitwise_and(segmented_image, segmented_image, mask=component_mask)
            
            if is_circle(component):
                segmented_image_colored[labels_im == label] = [0, 255, 0]  # Green for 'O'
            else:
                segmented_image_colored[labels_im == label] = [0, 0, 255]  # Red for 'X'
        
        cv_image_with_lines = cv2.polylines(cv_image.copy(), [pts], isClosed=True, color=(0, 255, 0), thickness=2)
        
        output_image = cv_image_with_lines.copy()
        output_image[mask == 255] = segmented_image_colored[mask == 255]
        
        processed_image_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
        
        self.publisher_.publish(processed_image_msg)
        self.get_logger().info('Published processed image')
        
        # Store the latest data for the service callback
        self.latest_image = output_image
        self.latest_labels = labels_im
        self.latest_num_labels = num_labels
        self.latest_largest_label = largest_label
        self.latest_pts = pts
        self.latest_segmented_image = segmented_image

    def get_board_state_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = "No image processed yet"
            return response

        board_state = get_board_state(
            self.latest_labels,
            self.latest_num_labels,
            self.latest_largest_label,
            self.latest_pts,
            self.latest_segmented_image
        )

        response.board_state = board_state.flatten().tolist()
        response.success = True
        response.message = "Board state retrieved successfully"
        return response

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