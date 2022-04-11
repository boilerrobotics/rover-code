'''
===============================================================================
Program Description 
	AR tag detection for minirover.

Author:         Thirawat Bureetes(Thirawat.tam@gmail.com)
Maintainer:     Thirawat Bureetes(Thirawat.tam@gmail.com)
Update:         April 11, 2022
Version:        0.1.0
===============================================================================
'''

import rclpy
from rclpy.node import Node
import cv2 as cv
import cv2.aruco as aruco

def parse_coords(corners):
    coords = []
    for cd in corners:
        x = int(str(cd[0]).split('.')[0])
        y = int(str(cd[1]).split('.')[0])
        coords.append([x, y])
    return coords




def main(args=None):
    rclpy.init(args=args)

    cap = cv.VideoCapture(0)

    while (True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Our operations on the frame come here
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            print('------------')
            ids = tuple(ids.tolist())
            coords = parse_coords(list(tuple(corners)[0][0]))
            tagmid = (coords[0][0] + coords[1][0] + coords[2][0] + coords[3][0]) / 4
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

    rclpy.shutdown()


if __name__ == '__main__':
    main()