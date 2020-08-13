from django.shortcuts import render
from django.http import HttpResponse

import time
from timeit import default_timer as timer 
from django.http import StreamingHttpResponse
from django.utils.timezone import now

import json
import uuid
import sys
import socket
import cv2
import pickle
import numpy as np
import struct 
import zlib
import logging
import random
import datetime
import numba

logging.basicConfig(level=logging.DEBUG)

'''================ Templates ===================-'''
# Drone Server
def home(request):
    return render(request,'index_gcs.html')
# Infomation
def about(request):
    return render(request,'about.html')
# Chart stream data
def chart(request):
    return render(request,'chart_data.html')   
# Test mapbox 
def mapbox(request):
    return render(request, 'mapbox.html')
# Drone ontrol system
def drone_control(request):
    return render(request, 'drone_control_system.html')


'''=============== Get stream data ============='''
# Server-Sent Events
def eventsource(request):
    # -- Socket UDP --
    def event_stream():               
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        sock.bind(('', 9999))        
        logging.info('Map server start.')                          
        while 1:            
            msg, adr = sock.recvfrom(1024) # type(msg):str
            msg = msg.decode('utf-8')
            if not msg:
                print('not msg')
                continue                                                
            else:
                logging.debug(">> Client {} send {}: ".format(adr,msg))                                             
                yield "data:{0}\n\n".format(msg)                         
                               
    return StreamingHttpResponse(event_stream(), content_type='text/event-stream')

def web_cam(request):       
    # -- Socket TCP --     
    def images_stream():        
        while True:
            s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)           
            s.bind(('',9998))    
            s.listen(1)
            logging.info('Cam server now listening ...')
            conn, addr = s.accept()
            logging.info("Client Info:{}{}".format(conn, addr))        
            data = b""
            payload_size = struct.calcsize(">L")
            logging.debug("payload_size: {}".format(payload_size))
            while True:
                try:
                    t = timer()
                    while len(data) < payload_size:                
                        if timer()-t > 0.25:                            
                            logging.warning(' >> time out')
                            break        
                        logging.debug("Recv: {}".format(len(data)))
                        #data += conn.recv(1024)
                        data += conn.recv(4096)
                        
                    logging.debug("Done Recv: {}".format(len(data)))                    
                                  
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack(">L", packed_msg_size)[0]
                    logging.debug("msg_size: {}".format(msg_size))
                    t = timer()
                    while len(data) < msg_size:
                        if timer()-t > 0.25:                            
                            logging.warning(' >> time out')
                            break
                        #data += conn.recv(1024)
                        data += conn.recv(4096)
                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    _, frame = cv2.imencode('.jpg', frame)                                
                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + (frame.tobytes()) + b'\r\n\r\n')
                except Exception as e:
                    logging.warning(str(e))
                    s.close()
                    break
                
    return StreamingHttpResponse(images_stream(),
                     content_type='multipart/x-mixed-replace; boundary=frame') 

def chart_data_eventsource(request):    
    def generate_random_data():
        while True:
            json_data = json.dumps(
                {'time': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 'value': random.random() * 100})
            yield f"data:{json_data}\n\n"
            time.sleep(1)

    return StreamingHttpResponse(generate_random_data(), content_type='text/event-stream')




''' def web_cam(request):  
    # -- Socket UDP fail--          
    def images_stream(): 
        while 1:              
            s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    
            s.bind(('',9998))   
            print('\n**********\nCam server now listening ...')
            while 1:                                                       
                data = b""
                payload_size = struct.calcsize(">L")
                no_data_occur = 0
                print("payload_size: {}".format(payload_size))            
                try:                    
                    while len(data) < payload_size:                        
                        print("Recv: {}".format(len(data)))
                        recieve_data, adr = s.recvfrom(4096) 
                        data += recieve_data   
                        if len(data) == 0:                                                 
                            no_data_occur += 1
                        if no_data_occur  >= 3:  
                            logging.INFO('\nwebcam client is disconnected ')                   
                            s.close()     
                            break           
                        print(data)                                             
                        
                    print("Done Recv: {}".format(len(data)))                                        
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack(">L", packed_msg_size)[0]
                    print("msg_size: {}".format(msg_size))
                    while len(data) < msg_size:
                        recieve_data, adr = s.recvfrom(4096)
                        data += recieve_data
                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    _, frame = cv2.imencode('.jpg', frame)                                
                    yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + (frame.tobytes()) + b'\r\n\r\n')
                except Exception as e:                
                    print(e)
                    
    return StreamingHttpResponse(images_stream(),
                        content_type='multipart/x-mixed-replace; boundary=frame')  '''


                     
# -- TCP --   
""" def eventsource(request):
    def event_stream():               
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 9999))
        sock.listen(5)
        print('\n**********\nMap server start, and wait for connected.')
        (csock, adr) = sock.accept()
        print ("\n**********\nClient Info: ", csock, adr) 
        client = csock         
        while 1:            
            msg = client.recv(1024).decode('utf-8') # type(msg):str
            if not msg:
                print("----------------------------------------------")
                print(f'client {adr} closed.')  
                print('waiting for connect.')
                (csock, adr) = sock.accept()
                print ("Client Info: ", csock, adr) 
                client = csock                              
            else:
                print ("Client send: " + msg)                                             
                yield "data:{0}\n\n".format(msg)                         
                               
    return StreamingHttpResponse(event_stream(), content_type='text/event-stream') """




