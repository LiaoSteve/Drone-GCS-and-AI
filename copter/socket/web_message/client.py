import socket, sys, time

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1',9999))
while 1:
    sock.sendall("Hello I'm Client 2.".encode('utf-8'))
    print(sock.recv(1024).decode('utf-8'))
    time.sleep(1)
sock.close()

        