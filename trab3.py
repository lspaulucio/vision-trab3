# -*- coding: utf-8 -*-

"""Trabalho 3 - Entrega até 10/07

Neste trabalho vocês deverão detectar o robô nos vídeos das 4 câmeras do espaço inteligente e obter a reconstrução
da  sua posição 3D no mundo. Feito isso, vocês deverão gerar um gráfico da posição do robô, mostrando a trajetória
que ele realizou.

Para detectar o robô será usado um marcador ARUCO acoplado a sua plataforma. Rotinas de detecção desse tipo de marcador
poderão ser usadas para obter sua posição central, assim como as suas quinas nas imagens. Essas informações, juntamente
com os dados de calibração das câmeras, poderão ser usadas para localização 3D do robô.

Informações a serem consideradas:
- O robô está identificado por um marcador do tipo ARUCO - Código ID 0 (zero) - Tamanho 30 x 30 cm
- Os vídeos estão sincronizados para garantir que a cada quadro vocês estarão processando imagens do robô capturadas no mesmo instante.
- A calibração das câmeras é fornecida em 4 arquivos no formato JSON (Para saber como ler os dados, basta procurar no Google).
- Rotinas de detecção dos marcadores Aruco em imagens e vídeo são fornecidas logo abaixo.

ATENÇĂO: Conforme comentado em sala de aula, existem rotinas de detecção de ARUCO que já fornecem sua localização e orientação 3D,
se a calibração da câmera e o tamanho do padrão forem fornecidas. Essas rotinas poderão ser usadas para fazer comparações com a
reconstrução 3D fornecida pelo trabalho de vocês, mas não serão aceitas como o trabalho a ser feito. Portanto, lembrem-se que vocês
deverão desenvolver a rotina de reconstrução, a partir da detecção do ARUCO acoplado ao robô nas imagens 2D capturadas nos vídeos.
"""


# [] are for JSON arrays, which are called list in Python
# {} are for JSON objects, which are called dict in Python

from Camera import Camera
import numpy as np


# cam0 = Camera('camera/0.json')
# cam1 = Camera('camera/1.json')
# cam2 = Camera('camera/2.json')
# cam3 = Camera('camera/3.json')
#
# print(np.linalg.inv(cam0.getExtrinsicMatrix()))

import cv2
import numpy as np
from cv2 import aruco

file_name = "videos/camera-00.mp4"
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
vid = cv2.VideoCapture(file_name)
cam = Camera('camera/3.json')

while True:
    _, img = vid.read()
    if img is None:
        print("Empty Frame")
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if len(corners) != 0:
        centroid = np.mean(corners[0], axis=1)
    frame = img.copy()
    frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.3, cam.getIntrinsicMatrix(), cam.getDistortion())

    if ids is not None:
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, cam.getIntrinsicMatrix(), cam.getDistortion(), rvec[i], tvec[i], 0.1)

    cv2.imshow('output', frame_markers)
    cv2.waitKey()
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
