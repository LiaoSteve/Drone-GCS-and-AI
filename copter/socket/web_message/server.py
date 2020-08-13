import socket, sys, json
import threading

def threadWork(client,adr):
    try:
        while True:
            msg = client.recv(1024).decode('utf-8')
            if not msg:
                print("----------------------------------------------")
                print(f'client {adr} closed')
                print("----------------------------------------------")
                break
            else:
                print ("Client send: " + msg)                
                client.sendall(("You say: " + msg).encode('utf-8'))                 
                try:
                    data = json.loads(msg) 
                    if data['channel'] == '00001':
                        print('name: {}'.format(data['name']))
                    if data['channel'] == '00002':
                        print('name: {}'.format(data['name']))  
                except Exception as e:                    
                    print(e)
                
    except Exception as e:
        print("----------------------------------------------")
        print(e)
        print('client {} closed'.format(adr))    
        print("----------------------------------------------")    
        client.close()         

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', 9999))
sock.listen(5)
print('server start ...')

while True:
    (csock, adr) = sock.accept()
    print ("Client Info: ", csock, adr)    
    t = threading.Thread(target=threadWork, args=(csock,adr), daemon=True)    
    t.start()
    
sock.close()