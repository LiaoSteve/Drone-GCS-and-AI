import socket, sys, time, json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1',9999))
data = {}
data["channel"] = "00002"
data["name"] = "GG"
while 1:
    sock.sendall(json.dumps(data).encode('utf-8'))
    print(sock.recv(1024).decode('utf-8'))
    time.sleep(1)
sock.close()
