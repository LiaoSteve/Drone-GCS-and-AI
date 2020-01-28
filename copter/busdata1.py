import json
from datetime import datetime
import uuid
import time
import sys
import socket

#READ COORDINATES FROM GEOJSON
input_file = open('./data/bus1.json')
json_array = json.load(input_file)
coordinates = json_array['features'][0]['geometry']['coordinates']

#GENERATE UUID
def generate_uuid():
    return uuid.uuid4()
#CONSTRUCT MESSAGE AND SEND IT TO KAFKA



def generate_checkpoint(sock,coordinates):
    i = 0
    while i < len(coordinates):
        data = {}
        data['channel'] = '00002'
        data['key'] = data['channel'] + '_' + str(generate_uuid())
        data['timestamp'] = str(datetime.utcnow())
        data['latitude'] = coordinates[i][1]
        data['longitude'] = coordinates[i][0]
        message = json.dumps(data)          
        sock.sendall(message.encode('utf-8'))
        print(message)
        #print(sock.recv(1024).decode('utf-8'))
        time.sleep(0.1)
        data = {}
        data['channel'] = '00001'
        message = json.dumps(data)     
        print(message)     
        sock.sendall(message.encode('utf-8'))
        #if bus reaches last coordinate, start from beginning
        if i == len(coordinates)-1:
            i = 0
        else:
            i += 1
if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('127.0.0.1',9999))
    try:
        generate_checkpoint(sock,coordinates)
    except:
        sys.exit(0)

