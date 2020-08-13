# client.py

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
    client = socket(AF_INET, SOCK_STREAM)  # 建立TCP套接字

    # 登錄函數
    if login(client):
        # 連接函數
        connect(client)

def connect(client):
    'connect 客戶端連接函數'

    # 使用select模塊的select方法實現IO多路複用監聽傳輸
    rlist = [client, sys.stdin]
    wlist = []
    xlist = []

    while True:
        rs, ws, xs = select(rlist, wlist, xlist)

        for r in rs:
            if r is client:
                # 接受服務器發來的消息
                data = client.recv(buffersize)
                if data.decode() == '\n':
                    # 如果消息爲回車，聊天室關閉
                    client.close()
                    print(txt_administrator_close_chatroom)
                    os._exit(0)
                else:
                    # 打印接收到的信息
                    print(data.decode(), end='')
            elif r is sys.stdin:
                # 發送消息給服務器
                data = sys.stdin.readline()
                if data == '\n':
                    # 如果回車，發送退出消息，關閉客戶端，退出聊天室
                    data = curuser + ': ' + txt_user_quit_chatroom + '\n'
                    client.send(data.encode())
                    client.close()
                    os._exit(0)
                else:
                    # 發信息給服務器
                    data = curuser + ': ' + data
                    client.send(data.encode())

def login(client):
    '登錄函數 login'
    # 使用全局變量管理用戶
    # 先讓客戶端輸入姓名
    global curuser
    curuser = input(txt_username)
    # 再連接到服務器，傳送用戶名以檢驗
    client.connect(ADDR)  # 連接到服務器地址
    print(txt_connect_to, ADDR)
    client.send(curuser.encode())
    data = client.recv(buffersize)
    if data.decode() == 'Username already exists!':
        # 如果用戶名已經存在，要求重新輸入
        print(txt_user_already_exists)
        return False
    else:
        # 發送信息給服務器，告知服務器用戶進入聊天室
        # -*- 因爲監聽傳輸的是sys.stdin.readline()，所以必須在最後添加換行符，以便清除阻塞 -*-
        data = curuser + ': ' + txt_uesr_enter_chatroom + '\n'
        client.send(data.encode())
        return True


if __name__ == '__main__':
    main()