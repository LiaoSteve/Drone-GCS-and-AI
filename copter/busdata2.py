import json
from datetime import datetime
import uuid
import time
import sys
import socket
import sys
import requests
#READ COORDINATES FROM GEOJSON
input_file = open('./data/bus1.json')
json_array = json.load(input_file)
coordinates = json_array['features'][0]['geometry']['coordinates']

#GENERATE UUID
def generate_uuid():
    return uuid.uuid4()
#CONSTRUCT MESSAGE AND SEND IT TO KAFKA
data = {}
data['channel'] = '00001'

def generate_checkpoint(coordinates):
    i = 0
    while i < len(coordinates):
        data['key'] = data['channel'] + '_' + str(generate_uuid())
        data['timestamp'] = str(datetime.utcnow())
        data['latitude'] = coordinates[i][1]
        data['longitude'] = coordinates[i][0]
        requests.post('127.0.0.1:8000', data=json.dumps(data))       
        
        
        #print(sock.recv(1024).decode('utf-8'))
        time.sleep(1)

        #if bus reaches last coordinate, start from beginning
        if i == len(coordinates)-1:
            i = 0
        else:
            i += 1
if __name__ == '__main__':    
    generate_checkpoint(coordinates)
    
        

