import cv2
import io
import socket
import struct
import time
import pickle
import zlib

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#client_socket.connect(('140.121.130.133',9102)) #
client_socket.connect(('140.121.130.97',6666)) # GTX 850M notebook
#client_socket.connect(('140.121.130.97',9102)) # gpu computer vs external network
connection = client_socket.makefile('wb')

cam = cv2.VideoCapture(0)
cam.set(3, 640)
cam.set(4, 480)

img_counter = 0

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
try:
    while True:
        ret, frame = cam.read()
        result, frame = cv2.imencode('.jpg', frame, encode_param)
    #    data = zlib.compress(pickle.dumps(frame, 0))
        data = pickle.dumps(frame, 0)
        size = len(data)


        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1
except Exception as e:
    cam.release()