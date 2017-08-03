---
layout: post
title: Linux에서 외부 실행 후 대기하기
category: C++
tag: [C++, Linux]
---

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
    wait(0);
    printf("The child just ended\n");
  }

  return 0;
}
</pre>