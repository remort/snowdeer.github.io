---
layout: post
title: Ctrl + C 키 이벤트 가로채기
category: C++
tag: [C++11, Event]
---

Linux에서 작동하는 코드입니다. (Cygwin에서 빌드하면 윈도우에서도 작동합니다.)

기본적으로 <kbd>Ctrl</kbd> + <kbd>C</kbd> 키를 누르게 되면 프로그램이 강제로 종료되는데,
프로그램이 종료되기 전에 뭔가 처리해야 할 요소가 있다면 아래의 코드를 이용해서
<kbd>Ctrl</kbd> + <kbd>C</kbd> 키 이벤트를 가로챌 수 있습니다.


## 예제 코드

<pre class="prettyprint">#include &lt;cstdio&gt;
#include &lt;sys/stat.h&gt;
#include &lt;cstdlib&gt;
#include &lt;unistd.h&gt;

void (*breakCapture)(int);

void signalingHandler(int signo) {
  printf("'Ctrl + C' processing...");

  exit(1);
}

using namespace std;

int main(void) {
  printf("Hello SnowDeer.\n");

  setsid();
  umask(0);

  breakCapture = signal(SIGINT, signalingHandler);

  while(true) {
    printf("Hello...\n");
    sleep(1);
  }

  return 0;
}</pre>


## 실행 화면

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-18-cpp11-ctrl-c-event-hooking/01.png)
