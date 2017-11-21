---
layout: post
title: BeautifulSoup 라이브러리 활용 웹페이지 파싱(Parsing)
category: Python
tag: [Python]
---

Python 3.x 기반에서 동작하는 TCP 기반의 Echo Server/Client 예제입니다.

# TCP Echo Server

<pre class="prettyprint">
import socketserver
import sys

class TcpEchoServerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        print('Client is connected : {0}'.format(self.client_address[0]))
        sock = self.request

        buffer = sock.recv(1024)
        received_message = str(buffer, encoding='utf-8')
        print('Received : {0}'.format(received_message))
        sock.send(buffer)
        sock.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('{0} <Bind IP>'.format(sys.argv[0]))
        sys.exit()

    bindIP = sys.argv[1]
    bindPort = 10070

    server = socketserver.TCPServer((bindIP, bindPort), TcpEchoServerHandler)

    print("Start Echo-Server")
    server.serve_forever()
</pre>

<br>

# TCP Echo Client

<pre class="prettyprint">
import socket
import sys

serverPort = 10070

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print('{0} <BindIP> <Server IP> <Message>'.format(sys.argv[0]))
        sys.exit()

    bindIP = sys.argv[1]
    serverIP = sys.argv[2]
    message = sys.argv[3]

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((bindIP, 0))

    try:
        sock.connect((serverIP, serverPort))

        buffer = bytes(message, encoding='utf-8')
        sock.send(buffer)
        print('Sended message : {0}'.format(message))

        buffer = sock.recv(1024)
        received_message = str(buffer, encoding='utf-8')
        print('Received message : {0}'.format(received_message))

    finally:
        sock.close()
</pre>