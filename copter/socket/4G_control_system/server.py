# server.py

# 導入系統模塊
import os, sys
# 導入網絡編程（傳輸層）模塊
from socket import *
# IO多路複用模塊
from select import select
# 設置模塊
from settings import *
# 語言模塊
from language import *

def main():
    'main 主函數'
    server = socket(AF_INET, SOCK_STREAM)  # 建立TCP套接字
    server.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)  # 設置端口可立即重用
    server.bind(ADDR)  # 綁定地址
    server.listen()  # 監聽

    # 接收函數
    accept(server)

def accept(server):
    'accept 服務器接受函數'

    # 使用select模塊的select方法實現IO多路複用監聽傳輸
    rlist = [server, sys.stdin]
    wlist = []
    xlist = []

    while True:
        rs, ws, xs = select(rlist, wlist, xlist)

        for r in rs:
            if r is server:
                # 服務器接受客戶端連接
                conn, addr = server.accept()
                # 調用validate函數檢查用戶名
                if validate(conn):
                    # 將客戶端套接字添加到rlist中以監聽
                    rlist.append(conn)
                    # 如果用戶名註冊成功
                    print(txt_connect_from, addr)
                else:
                    conn.close()
            elif r is sys.stdin:
                # 服務器向所有客戶端發送系統（管理員）消息
                data = sys.stdin.readline()
                if data == '\n':
                    # 如果服務器輸入回車，則退出
                    for c in rlist[2:]:
                        c.send(b'\n')
                        c.close()
                    server.close()
                    print(txt_administrator_close_chatroom)
                    os._exit(0)
                else:
                    # 如果服務器輸入正常語句，通知所有客戶端
                    data = administrator + ': ' + data
                    for c in rlist[2:]:
                        c.send(data.encode())
            else:
                # 服務器接受客戶端的消息並轉發給所有客戶端
                data = r.recv(buffersize)
                if not data:
                    # 關閉客戶端
                    r.close()
                    rlist.remove(r)
                else:
                    # 轉發信息給其他客戶端
                    print(data.decode(), end='')
                    for c in rlist[2:]:
                        if c is not r:
                            c.send(data)

def validate(client):
    '檢驗用戶名 validate username'
    name = client.recv(buffersize).decode()
    # print(name.decode())
    # print(users)
    if name in users:
        client.send(b'Username already exists!')
        return False
    else:
        users.append(name)
        client.send(b'Welcome!')
        return True


if __name__ == '__main__':
    # 全局變量，管理用戶信息
    users = []

    # 主函數
    main()