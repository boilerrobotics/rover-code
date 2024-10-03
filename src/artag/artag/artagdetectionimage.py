import numpy as np
import cv2 as cv

# Function to draw hierarchy lines
def draw_hierarchy_lines(contours, hierarchy, parent_idx, color, image):
    for i in range(len(contours)):
        if hierarchy[0][i][3] == parent_idx:
            cv.drawContours(image, contours, i, color, 2, cv.LINE_AA)
            
def straighten_contours(contours, epsilon):
    straightened_contours = []
    for contour in contours:
        approx = cv.approxPolyDP(contour, epsilon, True)
        straightened_contours.append(approx)
    return straightened_contours

# Function that filters contours based on area
def filter_contours(contour):
    filtered_contours = []
    min_area_threshold = 150  # Adjust this value as needed
    for contour in contours:
        area = cv.contourArea(contour)
        if min_area_threshold < area:
            filtered_contours.append(contour)
    return filtered_contours

# read input image in input folder
image = cv.imread('src\\artag\\artag\\input\\image_1.jpg')

# convert the image to grayscale format
img_gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# apply blurring to image to remove noise
img_gray = cv.blur(img_gray, (5, 5))
img_gray = cv.GaussianBlur(img_gray, (5, 5), 0)
img_gray = cv.medianBlur(img_gray, 5)
img_gray = cv.bilateralFilter(img_gray,9,75,75)

# apply binary thresholding
ret, thresh = cv.threshold(img_gray, 150, 255, cv.THRESH_BINARY)

# visualize the binary image before contours
cv.imshow('Binary image', thresh)
cv.waitKey(0)
cv.imwrite('image_thres1.jpg', thresh)
cv.destroyAllWindows()

# detect the contours on the binary image using cv.CHAIN_APPROX_NONE

contours, hierarchy = cv.findContours(image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)

# Draw contours on the original image
image_copy = image
contours = filter_contours(contours)
contours = straighten_contours(contours, .99)
# Draw straightened contours
for i, contour in enumerate(contours):    
    parent_idx = hierarchy[0][i][3]  # Get the index of the parent contour
    if parent_idx == -1:
        cv.drawContours(image=image_copy, contours=[contour], contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
    else:
        if parent_idx == 1:
            draw_hierarchy_lines(contours, hierarchy, parent_idx, (255, 0, 0), image_copy)
        else:
            draw_hierarchy_lines(contours, hierarchy, parent_idx, (0, 0, 255), image_copy)

# Calculate the center coordinates of the largest contour
if len(contours) > 0:
    largest_contour = max(contours, key=cv.contourArea)
    M = cv.moments(largest_contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    # Draw a circle at the center of the contour
    cv.circle(image_copy, (cx, cy), 5, (0, 0, 255), -1)
    # Print the coordinates of the center
    print("Center coordinates (x, y):", cx, cy)

# see the results
cv.imshow('None approximation', image_copy)
cv.waitKey(0)
cv.imwrite('contours_none_image1.jpg', image_copy)
cv.destroyAllWindows()