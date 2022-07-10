import numpy as np
import cv2

"""
Upon showing 5 different 4x4 aruco tags, it will output its ID and coordinates on the frame it was detected on
"""

cv2.__version__
import cv2.aruco as aruco

# import cv_bridge.aruco as aruco

def parse_coords(corners):
    coords = []
    for cd in corners:
        x = int(str(cd[0]).split('.')[0])
        y = int(str(cd[1]).split('.')[0])
        coords.append([x, y])
    return coords

cap = cv2.VideoCapture(1)
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
    # frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    # For a single marker
    # markerLength = 1
    # rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)

    # x = (corners[i-1][0][0][0] + corners[i-1][0][1][0] + corners[i-1][0][2][0] + corners[i-1][0][3][0]) / 4
    # y = (corners[i-1][0][0][1] + corners[i-1][0][1][1] + corners[i-1][0][2][1] + corners[i-1][0][3][1]) / 4
    # rotM = np.zeros(shape=(3,3))
    # cv2.Rodrigues(rvec[i-1], rotM, jacobian = 0)
    # ypr = cv2.RQDecomp3x3(rotM)

    if ids is not None:
        print('------------')
        ids = tuple(ids.tolist())
        coords = parse_coords(list(tuple(corners)[0][0]))
        # print(coords)
        tagmid = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) / 4
        # print(tagmid)
        # if ids not in id_set:
        #     if(tagmid >= 220 and tagmid < 440):
        #         id_set.append(ids)
        #         corner_set.append(tuple(corners))
        if tagmid < 220:
            seg = -1
            print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
        elif (tagmid >= 220 and tagmid < 440):
            widthperc1 = (abs(coords[0][1] - coords[1][1]) / coords[0][1] + abs(coords[0][1] - coords[1][1]) / coords[1][1]) / 2
            widthperc2 = (abs(coords[2][1] - coords[3][1]) / coords[2][1] + abs(coords[2][1] - coords[3][1]) / coords[3][1]) / 2
            heightperc1 = (abs(coords[0][0] - coords[3][0]) / coords[0][0] + abs(coords[0][0] - coords[3][0]) /
                          coords[3][0]) / 2
            heightperc2 = (abs(coords[1][0] - coords[2][0]) / coords[1][0] + abs(coords[1][0] - coords[2][0]) /
                          coords[2][0]) / 2
            try:
                genperc1 = (abs(abs(coords[0][0] - coords[1][0]) - abs(coords[0][1] - coords[3][1])) / abs(coords[0][0] - coords[1][0]) + abs(abs(coords[0][0] - coords[1][0]) - abs(coords[0][1] - coords[3][1])) / abs(coords[0][1] - coords[3][1])) / 2
            except ZeroDivisionError:
                genperc1 = 0
            try:
                genperc2 = (abs(abs(coords[2][0] - coords[3][0]) - abs(coords[1][1] - coords[2][1])) / abs(
                    coords[2][0] - coords[3][0]) + abs(
                    abs(coords[2][0] - coords[3][0]) - abs(coords[1][1] - coords[2][1])) / abs(
                    coords[1][1] - coords[2][1])) / 2
            except ZeroDivisionError:
                genperc2 = 0
            seg = 0
            if all(x < 0.05 for x in [widthperc1, widthperc2, heightperc1, heightperc2, genperc1, genperc2]):
                tagside = 5 # cm
                tagarea = tagside**2
                captureside = abs(coords[0][0] - coords[1][0]) * 0.0264583333
                coeff = tagside * 18.1 - 6
                distance = coeff / captureside
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}\nDistance: {distance} cm")
            else:
                print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
        else:
            seg = 1
            print(f"ID: {ids[0][0]}\nCoords: {coords}\nSegment: {seg}")
    # if len(id_set) == 5:
    #     break

    # # print(ids)
    # # print(corners)
    # # print(x)
    # # print(y)
    # # print(ypr)

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
