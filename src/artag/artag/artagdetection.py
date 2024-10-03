import numpy as np
import cv2 as cv

# margin of error for centering robot with ar tag
margin_of_error = 10

# Function to draw hierarchy lines
def draw_hierarchy_lines(contours, hierarchy, parent_idx, color, image):
    for i, contour in enumerate(contours):
        if hierarchy[0][i-1][3] == parent_idx:
            cv.drawContours(image, contours, i, color, 2, cv.LINE_AA)
            
def straighten_contours(contours, epsilon):
    straightened_contours = []
    for contour in contours:
        # Approximate contour using the Ramer-Douglas-Peucker algorithm
        approx = cv.approxPolyDP(contour, epsilon, True)
        straightened_contours.append(approx)
    return straightened_contours

# Function that filters contours based on area
def filter_contour(contour):
    filtered_contours = []
    # filters out potential ar tags that are too big or small (typically false positives) however might not be robust enough
    min_area_threshold = 1000  # Adjust this value as needed
    max_area_threshold = 100000
    for contour in contours:
        area = cv.contourArea(contour)
        if min_area_threshold < area < max_area_threshold:
            filtered_contours.append(contour)
    return filtered_contours

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    ret, frame = cap.read()
    frame_height, frame_width = frame.shape[:2]
    if not ret:
        print("Can't receive frame")
        break
    # apply filters to frame
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.blur(gray, (5, 5))
    gray = cv.GaussianBlur(gray, (5, 5), 0)
    gray = cv.medianBlur(gray, 5)
    gray = cv.bilateralFilter(gray,9,75,75)
    ret, thresh = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)

    # finds contours and gets all square contours
    contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)
    contours = filter_contour(contours)
    square_contours = [c for c in contours if len(cv.approxPolyDP(c, 0.02*cv.arcLength(c, True), True)) == 4]
    inner_contours = []
    for square_contour in square_contours:
        inner = []
        for contour in contours:
            if contour is not square_contour:
                M = cv.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                else:
                    cx = 0
                    cy = 0
                # Check if the centroid lies inside the square contour
                if cv.pointPolygonTest(square_contour, (cx, cy), False) > 0:
                    inner.append(contour)
        inner_contours.extend(inner)
    if square_contours:
        contours = square_contours
        contours.extend(inner_contours)
    else:
        contours = square_contours

    cv.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
    # Calculate the center coordinates of the largest contour
    if len(contours) > 0:
        largest_contour = max(contours, key=lambda c: cv.contourArea(c) / (frame_width * frame_height))
        M = cv.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = 0
            cy = 0
        # Draw a circle at the center of the contour
        cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        # Print the coordinates of the center
        # Goal is to have robot turn based on if ar tag is left or right of it
        # Adjust margin of error (hasn't been fully tuned)
        if cx + margin_of_error < frame_width // 2:
            print("Left")
        elif cx - margin_of_error > frame_width // 2:
            print("Right")
        else:
            print("Center")

    
    cv.imshow('None approximation', frame)
    if cv.waitKey(1) == ord('q'):
        break