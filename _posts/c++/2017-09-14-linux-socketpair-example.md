---
layout: post
title: socketpair 예제
category: C++
tag: [C++]
---
# socketpair

주로 프로세스간 통신을 위해 소켓을 생성할 때 사용하는 방법 중 하나입니다. `socketpair()` 함수는 주소를 갖지 않는 한 쌍의 소켓을 생성해줍니다. 이 소켓들을 이용해서 부모 프로세스와 자식 프로세스간 통신을 수행할 수 있습니다.

<br>

## 예제 코드

예제 코드는 다음과 같습니다.

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;unistd.h&gt;
#include &lt;sys/socket.h&gt;
#include &lt;cstring&gt;
#include &lt;sys/wait.h&gt;

int main(int argc, char **argv) {
  int ret, socket_fd[2];
  char buffer[] = "hello. snowdeer.";
  char line[BUFSIZ];

  ret = socketpair(AF_LOCAL, SOCK_STREAM, 0, socket_fd);
  if (ret == -1) {
    perror("socketpair error!!");
    return -1;
  }

  printf("socket 1 : %d\n", socket_fd[0]);
  printf("socket 2 : %d\n", socket_fd[1]);

  pid_t pid;
  int status;
  if ((pid = fork()) < 0) {
    perror("fork error!!");
  } else if (pid == 0) {
    write(socket_fd[0], buffer, strlen(buffer) + 1);
    printf("Data send : %s\n", buffer);

    close(socket_fd[0]);
  } else {
    wait(&status);

    read(socket_fd[1], line, BUFSIZ);
    printf("Data received : %s\n", line);

    close(socket_fd[1]);
  }

  return 0;
}
</pre>