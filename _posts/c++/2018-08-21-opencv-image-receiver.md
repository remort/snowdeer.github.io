---
layout: post
title: OpenCV 이미지 Receiver 예제 (TCP 기반)
category: C++
tag: [C++]
---
# OpenCV 이미지 Receiver 예제 (TCP 기반)

OpenCV 이미지를 다운로드한다기보다는 일반적인 파일을 다운로드(TCP 기반) 하는 예제입니다.

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.12)
project(ImageTcpReceiver)

set(CMAKE_CXX_STANDARD 14)

add_executable(ImageTcpReceiver main.cpp ImageServer.cc ImageServer.h ClientHandlerThread.cc ClientHandlerThread.h)

target_link_libraries(ImageTcpReceiver pthread)
</pre>

<br>

## ImageServer.h

<pre class="prettyprint">
#ifndef IMAGETCPRECEIVER_IMAGESERVER_H
#define IMAGETCPRECEIVER_IMAGESERVER_H

#include "ClientHandlerThread.h"

#include &lt;thread&gt;
#include &lt;vector&gt;

using namespace std;

class ImageServer {
 public:
  ImageServer();
  ~ImageServer();

  void start();
  void stop();


 private:
  bool initAcceptSocket();
  void finAcceptSocket();

  void run();

  bool mIsRunning;
  thread mAcceptorThread;

  int mSocketId;
  vector&lt;shared_ptr&lt;ClientHandlerThread&gt;&gt; mClientList;

};

#endif //IMAGETCPRECEIVER_IMAGESERVER_H
</pre>

<br>

## ImageServer.cc

<pre class="prettyprint">
#include "ImageServer.h"

#include &lt;iostream&gt;
#include &lt;sys/socket.h&gt;
#include &lt;zconf.h&gt;
#include &lt;arpa/inet.h&gt;

const int PORT = 10050;

ImageServer::ImageServer() {
  mIsRunning = false;
  mSocketId = -1;
  mClientList.clear();
}

ImageServer::~ImageServer() {

}

void ImageServer::start() {
  cout << "ImageServer::start() " << endl;
  mAcceptorThread = std::thread(&ImageServer::run, this);
}

void ImageServer::stop() {
  cout << "ImageServer::stop() " << endl;
  mIsRunning = false;
  if (mAcceptorThread.joinable()) {
    mAcceptorThread.join();
  }
}

bool ImageServer::initAcceptSocket() {
  cout << "ImageServer::initAcceptSocket() " << endl;

  struct sockaddr_in loc_addr = {0};
  loc_addr.sin_family = AF_INET;
  loc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  loc_addr.sin_port = htons(PORT);

  mSocketId = socket(AF_INET, SOCK_STREAM, 0);

  int opt = 1;
  int error;
  error = setsockopt(mSocketId, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
  if (error < 0) {
    perror("AcceptSocket - setsockopt(mSocketId, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT) Failed");
    return false;
  }

  error = bind(mSocketId, (struct sockaddr *) &loc_addr, sizeof(loc_addr));
  if (error < 0) {
    return false;
  }

  error = listen(mSocketId, 1);
  if (error < 0) {
    return false;
  }

  cout << "Server Socker is binding to " << PORT << ". " << endl;

  return true;
}

void ImageServer::finAcceptSocket() {
  cout << "ImageServer::finAcceptSocket() " << endl;
  if (mSocketId > 0) {
    shutdown(mSocketId, SHUT_RDWR);
    close(mSocketId);
    mSocketId = -1;
  }
}

void ImageServer::run() {

  bool ret = initAcceptSocket();
  if (ret == false) {
    cout << "Creating Accept Socket is failed." << endl;
    return;
  }

  struct sockaddr_in client_address;
  socklen_t client_len = sizeof(client_address);
  int clientSocketId;

  mIsRunning = true;
  while (mIsRunning) {
    cout << "Waiting Client..." << endl;

    //clientSocketId = accept(mSocketId, (struct sockaddr *) &rem_addr, &opt);
    clientSocketId = accept(mSocketId, (struct sockaddr *) &client_address, &client_len);

    if (mIsRunning == false) {
      break;
    }

    if (clientSocketId > 0) {
      char clntName[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_address.sin_addr.s_addr, clntName, sizeof(clntName));

      cout << "Client is Connected. (" << clientSocketId << ")"<<endl;

      shared_ptr<ClientHandlerThread>
          clientHandler(new ClientHandlerThread(clientSocketId));
      clientHandler->start();
      mClientList.push_back(clientHandler);
    }

  }

  finAcceptSocket();
}
</pre>

<br>

## ClientHandlerThread.h

<pre class="prettyprint">
#ifndef IMAGETCPRECEIVER_CLIENTHANDLERTHREAD_H
#define IMAGETCPRECEIVER_CLIENTHANDLERTHREAD_H

#include &lt;thread&gt;

using namespace std;

class ClientHandlerThread {
 public:
  ClientHandlerThread(int socketId);
  virtual ~ClientHandlerThread();

  void start();
  void stop();

 private:
  void run();

  void savePacketAsFile(string filepath);

  thread mThread;
  bool mIsRunning;

  int mSocketId;
};

#endif //IMAGETCPRECEIVER_CLIENTHANDLERTHREAD_H
</pre>

<br>

## ClientHandlerThread.cc

<pre class="prettyprint">
#include "ClientHandlerThread.h"

#include &lt;iostream&gt;
#include &lt;sys/socket.h&gt;
#include &lt;sys/types.h&gt;
#include &lt;sys/time.h&gt;
#include &lt;zconf.h&gt;

ClientHandlerThread::ClientHandlerThread(int socketId) {
  mSocketId = socketId;
  mIsRunning = false;
  thread_local int optval = 1;
  setsockopt(mSocketId, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
}

ClientHandlerThread::~ClientHandlerThread() {

}

void ClientHandlerThread::start() {
  cout << "ClientHandlerThread::start() " << endl;
  mThread = thread(&ClientHandlerThread::run, this);
}

void ClientHandlerThread::stop() {
  cout << "ClientHandlerThread::stop() " << endl;
  mIsRunning = false;
}

void ClientHandlerThread::run() {
  mIsRunning = true;
  while (mIsRunning) {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

    char filename[256] = "";
    sprintf(filename, "image_%ld.jpg", ms);

    savePacketAsFile(filename);
  }
}

void ClientHandlerThread::savePacketAsFile(string filepath) {
  thread_local char packet[1024];

  long file_size = 0;
  read(mSocketId, &file_size, sizeof(long));

  if(file_size == 0) return;

  cout << "size: " << file_size << endl;
  
  FILE *file;
  file = fopen(filepath.c_str(), "wb+");

  int bytes_received = 0;
  int length;
  while (bytes_received < file_size) {
    //cout << "remains: " << remains << endl;
    int remains = file_size - bytes_received;

    int packetSize = sizeof(packet);
    if (remains < packetSize) {
      length = read(mSocketId, packet, remains);
    } else {
      length = read(mSocketId, packet, sizeof(packet));
    }

    bytes_received += length;
    fwrite(packet, length, 1, file);
  }

  fclose(file);
}
</pre>

<br>

## Main.cc

<pre class="prettyprint">
#include "ImageServer.h"

#include &lt;iostream&gt;
#include &lt;unistd.h&gt;

int main() {
  std::cout << "Hello, World!" << std::endl;

  shared_ptr&lt;ImageServer&gt; server = make_shared&lt;ImageServer&gt;();
  server->start();

  while(true) {
    usleep(1000 * 1000);
  }

  server->stop();

  return 0;
}
</pre>