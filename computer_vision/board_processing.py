import cv2
import numpy as np

def image_segmentation(img: np.ndarray, invert=False, roi=list()) -> tuple:
    """
    Obtain a segmentation of the elements in the board.

    Parameters:
    - invert: Use `True` if the background of your board is white.
    - roi: Define it as [x, y, w, h] if required.

    Returns:
    - img: Processed image.
    - num_labels: Number of labels found.
    - labels_im: Labeled image with connected components.
    """
    if len(roi) > 0:
        assert len(roi) == 4, "ROI should be defined as [x, y, w, h]"
        # Extract the ROI
        img = img[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
    
    # Apply binary threshold
    _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    # Invert the binary image
    if invert:
        img = cv2.bitwise_not(img)
    # Label connected components
    num_labels, labels_im = cv2.connectedComponents(img)

    return img, num_labels, labels_im

# Function to find the largest component
def find_largest_component(labels, num_labels):
    max_area = 0
    largest_label = 1
    for label in range(1, num_labels):
        area = np.sum(labels == label)
        if area > max_area:
            max_area = area
            largest_label = label
    return largest_label

def find_grid_intersections(grid_mask: np.ndarray) -> list:
    """
    Find the four intersections of a grid in a binary mask.

    Parameters:
    - grid_mask (np.ndarray): A binary image where the grid is white (255) and the background is black (0).

    Returns:
    - intersections (list): A list of tuples representing the four intersections of the grid.
    """
    try:
        # Horizontal traversal
        mid_y = grid_mask.shape[0] // 2
        transition_points_h = []
        previous_pixel = 0
        for x in range(grid_mask.shape[1]):
            current_pixel = grid_mask[mid_y, x]
            if previous_pixel == 0 and current_pixel == 255:
                transition_points_h.append((x, mid_y))
            previous_pixel = current_pixel
            if len(transition_points_h) == 2:
                break

        # Vertical traversal
        mid_x = grid_mask.shape[1] // 2
        transition_points_v = []
        previous_pixel = 0
        for y in range(grid_mask.shape[0]):
            current_pixel = grid_mask[y, mid_x]
            if previous_pixel == 0 and current_pixel == 255:
                transition_points_v.append((mid_x, y))
            previous_pixel = current_pixel
            if len(transition_points_v) == 2:
                break

        if len(transition_points_h) != 2 or len(transition_points_v) != 2:
            raise ValueError("Could not find exactly 4 transition points")

        # Generate lines and find intersections
        intersections = [
            (transition_points_h[0][0], transition_points_v[0][1]),
            (transition_points_h[1][0], transition_points_v[0][1]),
            (transition_points_h[0][0], transition_points_v[1][1]),
            (transition_points_h[1][0], transition_points_v[1][1])
        ]

        return intersections

    except Exception as e:
        return []
    
# Function to check if a component is a circle ('O')
def is_circle(component):
    x, y, w, h = cv2.boundingRect(component)
    mid_x = x + w // 2
    start_y = y + h // 3
    end_y = y + 2 * h // 3

    for i in range(start_y, end_y):
        if component[i, mid_x] == 255:
            return False

    return True

# Function to check if a component is an 'X'
def is_x(component):
    x, y, w, h = cv2.boundingRect(component)
    quarter_x = x + w // 3
    start_y = y
    end_y = y + h

    pattern_found = False
    for i in range(start_y, end_y):
        if component[i, quarter_x] == 255:  # Check for the first segment of pixels
            while i < end_y and component[i, quarter_x] == 255:
                i += 1
            if i < end_y and component[i, quarter_x] == 0:  # Check for the background segment
                while i < end_y and component[i, quarter_x] == 0:
                    i += 1
                if i < end_y and component[i, quarter_x] == 255:  # Check for the second segment of pixels
                    pattern_found = True
                    break

    return pattern_found

if __name__ == "__main__":
    # Open the camera stream
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera, or replace with the correct camera index

    # Set the ROI parameters
    x, y, w, h = 214, 192, 246, 244

    # Processing loop
    while cap.isOpened():
        # Check if frame is received
        ret, img = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Save original image
        original = np.copy(img)

        # Turn image to grayscale
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        processed_img, num_labels, labels_im = image_segmentation(img, True, [x, y, w, h])

        # Find the largest component
        if num_labels > 1:
            largest_label = find_largest_component(labels_im, num_labels)
            grid_mask = np.zeros(processed_img.shape, dtype=np.uint8)
            grid_mask[labels_im == largest_label] = 255
        
            # Highlight the largest component in blue within the ROI
            blue_component = np.zeros_like(original[y:y+h, x:x+w])
            blue_component[grid_mask == 255] = [255, 0, 0]  # Blue color
            original[y:y+h, x:x+w][grid_mask == 255] = blue_component[grid_mask == 255]

            # Find and draw intersections
            intersections = find_grid_intersections(grid_mask)
            if len(intersections) == 4:
                for (ix, iy) in intersections:
                    cv2.circle(original, (x+ix, y+iy), 5, (255, 0, 255), -1)  # Draw pink points
                cv2.putText(original, 'Board detected', 
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(original, 'Board not detected', 
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

            # Analyze remaining components
            for label in range(1, num_labels):
                if label == largest_label:
                    continue

                component_mask = np.zeros(processed_img.shape, dtype=np.uint8)
                component_mask[labels_im == label] = 255
                
                if is_circle(component_mask):
                    color = (0, 255, 0)  # Green color for 'O'
                elif is_x(component_mask):
                    color = (0, 0, 255)  # Red color for 'X'
                else:
                    continue

                # Color the component directly
                original[y:y+h, x:x+w][component_mask == 255] = color
                
        # Draw the ROI
        cv2.rectangle(original, (x, y), (x+w, y+h), (0, 0, 0), 5)

        # Display the number of components detected
        cv2.putText(original, f'Components detected: {num_labels - 1}', 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # Display the frame 
        cv2.imshow('Frame', original)

        # Exit loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

