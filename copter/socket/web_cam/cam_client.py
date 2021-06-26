import cv2
import io
import socket
import struct
import time
import pickle
import zlib

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 5]
while True:    
    try:         
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('140.121.130.133',9998)) #
        print('*********\nsocket connect OK')
        cam = cv2.VideoCapture(0)
        print('cam ok')
        cam.set(3, 640)
        cam.set(4, 480)        
        while 1:
            try:
                ret, frame = cam.read()
                result, frame = cv2.imencode('.jpg', frame, encode_param)            
                data = pickle.dumps(frame, 0)
                size = len(data)                      
                client_socket.sendall(struct.pack(">L", size) + data)                
            except Exception as e:
                print(e)
                cam.release()
                client_socket.close()
                break
    except Exception as e:
        time.sleep(2)
        print(e)    
        