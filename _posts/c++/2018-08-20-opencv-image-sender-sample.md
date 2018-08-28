---
layout: post
title: OpenCV 이미지 Sender 예제 (파일로 저장 후 TCP로 전송)
category: C++
tag: [C++]
---
# OpenCV 이미지 Sender 예제 (파일로 저장 후 TCP로 전송)

OpenCV 이미지를 파일에 저장한 다음 파일을 전송하는 예제 코드입니다.

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.12)
project(ImageTcpSender)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(ImageTcpSender main.cpp ImageSender.cc ImageSender.h)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
</pre>

<br>

## ImageSender.h

<pre class="prettyprint">
#ifndef IMAGETCPSENDER_IMAGESENDER_H
#define IMAGETCPSENDER_IMAGESENDER_H

#include &lt;string&gt;

using namespace std;

class ImageSender {
 public:
  ImageSender();
  ~ImageSender();

  void connectToServer(string ipAddress, int port);
  void disconnectFromServer();

  void sendImageFile(string filename);

 private:
  long getFileSize(std::string filename);

  int mSocketId;
};
</pre>

<br>

## ImageSender.cc

<pre class="prettyprint">
#include "ImageSender.h"

#include &lt;sys/socket.h&gt;
#include &lt;netinet/in.h&gt;
#include &lt;iostream&gt;
#include &lt;arpa/inet.h&gt;
#include &lt;zconf.h&gt;

ImageSender::ImageSender() {
  mSocketId = -1;
}

ImageSender::~ImageSender() {
}

void ImageSender::connectToServer(string ipAddress, int port) {
  struct sockaddr_in server_addr = {0,};
  char buf[1024];

  mSocketId = socket(AF_INET, SOCK_STREAM, 0);
  if (mSocketId < 0) {
    cout << "Error: Create client socket" << endl;
    return;
  }

  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = inet_addr(ipAddress.c_str());
  server_addr.sin_port = htons(port);

  int ret = connect(mSocketId, (struct sockaddr *) &server_addr, sizeof(server_addr));
  if (ret < 0) {
    cout << "Error: Connect to Server" << endl;
    return;
  }

}

void ImageSender::disconnectFromServer() {
  // TODO
}

long ImageSender::getFileSize(std::string filename) {
  FILE *f;
  f = fopen(filename.c_str(), "r");
  fseek(f, 0, SEEK_END);
  return (long) ftell(f);
}

long min(long a, long b) {
  if (a < b) return a;
  return b;
}

void ImageSender::sendImageFile(string filename) {
  if (mSocketId < 0) {
    cout << "There is no connection !!" << endl;
    return;
  }

  long file_size = getFileSize(filename);
  write(mSocketId, &file_size, sizeof(file_size));

  FILE *file = fopen(filename.c_str(), "rb");
  char packet[1024];
  long remains = file_size;
  while (remains > 0) {
    //cout << "remains: " << remains << endl;
    int packetSize = sizeof(packet);

    if (remains < packetSize) {
      fread(packet, 1, remains, file);
      write(mSocketId, &packet, remains);
    } else {
      fread(packet, 1, packetSize, file);
      write(mSocketId, &packet, packetSize);
    }

    remains -= packetSize;
  }
  fclose(file);

}
</pre>

<br>

## main.cc

<pre class="prettyprint">
#include "ImageSender.h"

#include &lt;opencv/cv.h&gt;
#include &lt;opencv/highgui.h&gt;
#include &lt;opencv2/videoio.hpp&gt;
#include &lt;cv.hpp&gt;
#include &lt;iostream&gt;
#include &lt;fstream&gt;
#include &lt;memory&gt;
#include &lt;sys/time.h&gt;
#include &lt;cstdio&gt;

using namespace std;

const int PORT = 10050;

string getFilenameFromTime() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;

  char filename[256] = "";
  sprintf(filename, "image_%ld.jpg", ms);

  return filename;
}

int main() {
  std::cout << "Hello, World!" << std::endl;

  auto sender = make_shared&lt;ImageSender&gt;();
  sender->connectToServer("127.0.0.1", PORT);

  cv::VideoCapture capture(0);
  if (!capture.isOpened()) {
    cout << "Failed to open Camera..." << endl;
    return -1;
  }

  while (true) {
    cv::Mat frame;

    try {
      capture >> frame;

      cv::imshow("Camera", frame);
      string filename = getFilenameFromTime();

      imwrite(filename.c_str(), frame);
      sender->sendImageFile(filename);
      remove(filename.c_str());

      if (cv::waitKey(30) == 27) break;
    } catch (cv::Exception &e) {
      cout << "Exception: " << e.err << endl;
    }

  }

  return 0;
}
</pre>