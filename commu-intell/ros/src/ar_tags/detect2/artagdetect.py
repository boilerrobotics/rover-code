import numpy as np
import cv2

"""
Upon showing 5 different 4x4 aruco tags, it will output its ID and coordinates on the frame it was detected on
"""

cv2.__version__
import cv2.aruco as aruco

# import cv_bridge.aruco as aruco

cap = cv2.VideoCapture(0)
detected_dict = {}
id_set = list()
corner_set = list()

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    # print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''

    # lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    # For a single marker
    # markerLength = 1
    # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)

    # x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
    # y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
    # rotM = np.zeros(shape=(3,3))
    # cv2.Rodrigues(rvec[i-1], rotM, jacobian = 0)
    # ypr = cv2.RQDecomp3x3(rotM)
    if ids is not None:
        ids = tuple(ids.tolist())
        if ids not in id_set:
            id_set.append(ids)
            corner_set.append(tuple(corners))

    if len(id_set) == 5:
        break

    # print(ids)
    # print(corners)
    # print(x)
    # print(y)
    # print(ypr)

    gray = aruco.drawDetectedMarkers(gray, corners)

    # print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame', gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

for x in range(len(id_set)):
    # print("Type: ", type(corner_set[x][0][0]))
    detected_dict[id_set[x][0][0]] = corner_set[x][0][0]
#print(detected_dict.keys())
#print(list(detected_dict.values())[0])
for i in range(len(detected_dict.keys())):
    print(f"ID: {list(detected_dict.keys())[i]}\nCoords: {list(detected_dict.values())[i]}")
