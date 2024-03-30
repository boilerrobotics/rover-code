import numpy as np
import cv2
 
# read the image
image = cv2.imread('src\\artag\\artag\input\image_1.jpg')

# convert the image to grayscale format
img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# apply blurring to image to remove noise
img_gray = cv2.blur(img_gray, (5, 5))
img_gray = cv2.GaussianBlur(img_gray, (5, 5), 0)
img_gray = cv2.medianBlur(img_gray, 5)
img_gray = cv2.bilateralFilter(img_gray,9,75,75)

# apply binary thresholding
ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

# visualize the binary image before contours
cv2.imshow('Binary image', thresh)
cv2.waitKey(0)
cv2.imwrite('image_thres1.jpg', thresh)
cv2.destroyAllWindows()

# detect the contours on the binary image using cv2.CHAIN_APPROX_NONE

contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
                                      
# Function to draw hierarchy lines
def draw_hierarchy_lines(contours, hierarchy, parent_idx, color):
    for i in range(len(contours)):
        if hierarchy[0][i][3] == parent_idx:
            cv2.drawContours(image_copy, contours, i, color, 2, cv2.LINE_AA)

# Function to turn contours into straight lines

def straighten_contour(contour):
    epsilon = 0.01 * cv2.arcLength(contour, True) # Adjust epsilon as needed
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return approx

# Filter contours based on area
filtered_contours = []
min_area_threshold = 150  # Adjust this value as needed
for contour in contours:
    area = cv2.contourArea(contour)
    if min_area_threshold < area:
        filtered_contours.append(contour)

# Draw contours on the original image
image_copy = image

# Draw straightened contours
for i, contour in enumerate(filtered_contours):
    straight_contour = straighten_contour(contour)
    
    parent_idx = hierarchy[0][i][3]  # Get the index of the parent contour
    if parent_idx == -1:
        cv2.drawContours(image=image_copy, contours=[straight_contour], contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
    else:
        if parent_idx == 1:
            draw_hierarchy_lines(contours, hierarchy, parent_idx, (255, 0, 0))
        else:
            draw_hierarchy_lines(contours, hierarchy, parent_idx, (0, 0, 255))

# Calculate the center coordinates of the largest contour
if len(contours) > 0:
    #largest_contour = max(contours, key=cv2.contourArea)
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    # Draw a circle at the center of the contour
    cv2.circle(image_copy, (cx, cy), 5, (0, 0, 255), -1)
    # Print the coordinates of the center
    print("Center coordinates (x, y):", cx, cy)

# see the results
cv2.imshow('None approximation', image_copy)
cv2.waitKey(0)
cv2.imwrite('contours_none_image1.jpg', image_copy)
cv2.destroyAllWindows()