---
layout: post
title: Linux에서 외부 명령어 실행하는 동안 대기하기
category: C++
tag: [C++, 리눅스]
---
# 외부 명령어 실행하는 동안 대기하기

리눅스에서 외부 명령어를 실행하는 함수는 `execlp` 입니다. 외부 명령어를 호출하기 때문에 해당 명령어가 어떻게 실행되고 있는지, 종료가 되었는지 알기가 힘든데 `fork()`와 `wait()`를 이용하면 외부 명령어의 종료 시점을 알 수 있습니다.

<pre class="prettyprint">
#include &lt;cstdio&gt;
#include &lt;cstdlib&gt;
#include &lt;unistd.h&gt;
#include &lt;sys/wait.h&gt;

int main(void) {
  int pid;

  pid = fork();

  if (pid < 0) {
    printf("A fork error has occurred.\n");
    exit(-1);
  }

  if (pid == 0) {
    printf("Forking is successful.\n");
    execlp("/bin/ls", "ls", nullptr);
    exit(127);
  } else {
    printf("This is the parent.\n");    
    int status;
    waitpid(pid, &status, 0);
    printf("The child just ended\n");
  }

  return 0;
}
</pre>

<br>

## system() 함수

`system()` 함수는 내부적으로 `fork()`, `exec()`, `waitpid()`으로 이루어져 있기 때문에 보다 쉽게 사용할 수 있습니다.

<pre class="prettyprint">
#include &lt;stdlib.h&gt;

int system(const char *cmd);
</pre>

명령이 정상적으로 수행될 경우 `127` 값을 리턴하며, 오류가 난 경우 `-1`을 리턴합니다. `system()` 함수의 내부는 다음 코드와 비슷합니다.

<pre class="prettyprint">
#include &lt;stdio&gt;
#include &lt;errno&gt;
#include &lt;unistd.h&gt;
#include &lt;sys/types.h&gt;
#include &lt;sys/wait.h&gt;

int system(const char *cmd) {
  pid_t pid;
  int status;

  if ((pid = fork()) < 0) {
    status = -1;
  }
  else if (pid == 0) {
    execl("/bin/sh", "sh", "-c", cmd, (char *) 0);
    _exit(127);
  } else {
    while(waitpid(pid, &status, 0) < 0) {
      if(errno != EINTR) {
        status = -1;
        break;
      }
    }
  }

  return status;
}
</pre>