from django.shortcuts import render
from django.http import HttpResponse

import time
from django.http import StreamingHttpResponse
from django.utils.timezone import now

import json
import uuid
import sys
import socket

# Create your views here.
def home(request):
    return render(request,'index_gcs.html')

def about(request):
    return render(request,'about.html')

def eventsource(request):
    def event_stream():               
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 9999))
        sock.listen(5)
        print('Server start, and wait for connected.')
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
                               
    return StreamingHttpResponse(event_stream(), content_type='text/event-stream')


