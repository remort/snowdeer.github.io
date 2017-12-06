---
layout: post
title: Thread 기반 TCP Server/Client 예제
category: Python
tag: [Python]
---
# Thread 기반의 TCP Server/Client

Thread 기반으로 동작하는 TCP Server/Client 예제입니다.

<br>

## Common.constant.py

<pre class="prettyprint">
IP_ADDRESS = '127.0.0.1'
PORT = 10034
</pre>

<br>

## server_main.py

<pre class="prettyprint">
from TcpServer import tcpServer

if __name__ == '__main__':
    print('TCP Server starts.')

    while True:
        print('&lt;Menu&gt;')
        print('   1. Start Server')
        print('   2. Stop Server')
        print('   99. Finish')

        menu = input('Input the menu: ')

        # print('   --> Your input is {}'.format(menu))

        if menu == '1':
            tcpServer.start_server()
        elif menu == '2':
            tcpServer.stop_server()
        elif menu == '99':
            print("Exit !!!")
            tcpServer.stop_server()
            break

</pre>

<br>

## TcpServer.tcpServer.py

<pre class="prettyprint">
import threading
import socketserver
import Common.constant as constant

TAG = 'TcpServer'

stdout_lock = threading.Lock()


class TcpServerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        with stdout_lock:
            print('Client({}) is connected.'.format(self.client_address[0]))
        sock = self.request

        while True:
            buffer = sock.recv(1024).strip()

            if not buffer:
                sock.close()
                with stdout_lock:
                    print('Client({}) is disconnected.'.format(self.client_address[0]))
                break

            received_message = str(buffer, encoding='utf-8')
            with stdout_lock:
                print('Received: {}'.format(received_message))


class TcpServerThread(threading.Thread):
    socketserver.TCPServer.allow_reuse_address = True
    server = None

    def __init(self):
        pass

    def run(self):
        if not self.server is None:
            self.server.shutdown()
            self.server = None

        self.server = socketserver.TCPServer(('', constant.PORT), TcpServerHandler)
        with stdout_lock:
            print('TcpServerThread is started.')
            print('Listening on port({})'.format(constant.PORT))
        self.server.serve_forever()
        with stdout_lock:
            print('TcpServerThread is terminated.')

    def stop(self):
        self.server.shutdown()
        self.server = None


server_thread = None


def start_server():
    stop_server()

    global server_thread
    server_thread = TcpServerThread()
    server_thread.start()


def stop_server():
    global server_thread
    if not server_thread is None:
        server_thread.stop()
        server_thread.join()
        server_thread = None
</pre>

<br>

## tcp_client.py

<pre class="prettyprint">
from TcpClient import tcpClient

if __name__ == '__main__':
    print('TCP Client starts.')

    while True:
        print('&lt;Menu&gt;')
        print('   1. Connect to Server')
        print('   2. Send a message to Server')
        print('   9. Disconnect from Server')

        print('   99. Finish')

        menu = input('Input the menu: ')

        # print('   --> Your input is {}'.format(menu))

        if menu == '1':
            tcpClient.connect()
        elif menu == '2':
            tcpClient.send_message('hello. snowdeer')
        elif menu == '9':
            tcpClient.disconnect()
        elif menu == '99':
            break
</pre>

<br>

## TcpClient.tcpClient.py

<pre class="prettyprint">
import threading
import time
import socket
import Common.constant as constant
from Common.log import Log as log

TAG = 'TcpClient'


class TcpClientThread(threading.Thread):
    sock = None

    def __init(self):
        pass

    def run(self):
        log.i(TAG, 'Start TcpClientThread.')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.connect((constant.IP_ADDRESS, constant.PORT))

    def stop(self):
        self.sock.close()

    def send_message(self, message):
        buffer = bytes(message, encoding='utf-8')
        self.sock.send(buffer)


client_thread = None


def connect():
    disconnect()

    global client_thread
    client_thread = TcpClientThread()
    client_thread.daemon = True
    client_thread.start()


def disconnect():
    global client_thread
    if not client_thread is None:
        client_thread.stop()
        client_thread.join()
        client_thread = None


def send_message(message):
    try:
        client_thread.send_message(message)
    except:
        print("Error to send a message !!")
</pre>