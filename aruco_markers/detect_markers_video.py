import cv2
import numpy as np
from cv2 import aruco

file_name = "videos/camera-01.mp4"
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
vid = cv2.VideoCapture(file_name)

while True:
    _, img = vid.read()
    if img is None:
        print("Empty Frame")
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if len(corners) != 0:
        centroid = np.mean(corners[0], axis=1)

    frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
    cv2.imshow('output', frame_markers)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
