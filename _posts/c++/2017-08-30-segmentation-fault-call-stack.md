---
layout: post
title: Segmentation Fault 오류시 CallStack 확인 방법
category: C++
tag: [C++, Debug, 라즈베리파이]
---
# Segmentation Fault

'Segmentation Fault' 오류는 잘못된 메모리 영역을 참조할 때 발생하는 오류입니다. 이 때 단순히
'Segmentation Fault' 오류 메세지만 출력하기 때문에 어느 부분에서 오류가 발생했는지 디버그하기가
까다롭습니다. 물론, 디버깅 툴을 통해서 `callstack`을 보면서 디버그를 할 수 있으면 좋겠지만,
크로스 컴파일 환경에서는 디버깅이 쉽지 않은 경우가 대부분입니다.

다음 예제 코드는 라즈베리파이에서 돌아가는 Segmentation Fault시 Callstack을 츨력하는 코드입니다.
ARM 계열 CPU를 타켓으로 했으며, x86이나 다른 계열 CPU에서는 코드에서

<pre class="prettyprint">
caller_address = (void *) uc->uc_mcontext.arm_pc;  // RIP: x86_64 specific     arm_pc: ARM
</pre>

부분을 수정하면 됩니다.

<br>

# 예제 코드

<pre class="prettyprint">
#include &lt;stdio.h&gt;
#include &lt;execinfo.h&gt;
#include &lt;signal.h&gt;
#include &lt;stdlib.h&gt;
#include &lt;unistd.h&gt;
#include &lt;cxxabi.h&gt;
#include &lt;stdio.h&gt;
#include &lt;stdlib.h&gt;
#include &lt;string&gt;
#include &lt;execinfo.h&gt;
#include &lt;signal.h&gt;
#include &lt;stdio.h&gt;
#include &lt;stdlib.h&gt;
#include &lt;string.h&gt;
#include &lt;ucontext.h&gt;
#include &lt;unistd.h&gt;

using namespace std;

typedef struct _sig_ucontext {
  unsigned long uc_flags;
  struct ucontext *uc_link;
  stack_t uc_stack;
  struct sigcontext uc_mcontext;
  sigset_t uc_sigmask;
} sig_ucontext_t;

void crit_err_hdlr(int sig_num, siginfo_t * info, void * ucontext) {
  void * array[50];
  void * caller_address;
  char ** messages;
  int size, i;
  sig_ucontext_t * uc;

  uc = (sig_ucontext_t *) ucontext;

  /* Get the address at the time the signal was raised */
  caller_address = (void *) uc->uc_mcontext.arm_pc;  // RIP: x86_64 specific     arm_pc: ARM

  fprintf(stderr, "\n");

  if (sig_num == SIGSEGV)
    printf("signal %d (%s), address is %p from %p\n", sig_num, strsignal(sig_num), info->si_addr,
           (void *) caller_address);
  else
    printf("signal %d (%s)\n", sig_num, strsignal(sig_num));

  size = backtrace(array, 50);
  /* overwrite sigaction with caller's address */
  array[1] = caller_address;
  messages = backtrace_symbols(array, size);

  /* skip first stack frame (points here) */
  for (i = 1; i < size && messages != NULL; ++i) {
    printf("[bt]: (%d) %s\n", i, messages[i]);
  }

  free(messages);

  exit(EXIT_FAILURE);
}

void installSignal(int __sig) {
  struct sigaction sigact;
  sigact.sa_sigaction = crit_err_hdlr;
  sigact.sa_flags = SA_RESTART | SA_SIGINFO;
  if (sigaction(__sig, &sigact, (struct sigaction *) NULL) != 0) {
    fprintf(stderr, "error setting signal handler for %d (%s)\n", __sig, strsignal(__sig));
    exit(EXIT_FAILURE);
  }
}

int crash() {
  char *s = "hello world";
  *s = 'H';

  printf("Crash~!!");
  return 0;
}

int foo4() {
  crash();
  return 0;
}

int foo3() {
  foo4();
  return 0;
}

int foo2() {
  foo3();
  return 0;
}

int foo1() {
  foo2();
  return 0;
}

int main(int argc, char *argv[]) {

  // For crashes, SIGSEV should be enough.
  installSignal(SIGSEGV);

  // But, you can install handlers for other signals as well:
  //installSignal(SIGBUS);
  //installSignal(SIGFPE);
  //installSignal(SIGILL);
  //installSignal(SIGINT);
  //installSignal(SIGPIPE);
  //installSignal(SIGQUIT);
  //installSignal(SIGTERM);
  //installSignal(SIGTSTP); # This is Ctrl+Z, we need to ignore it. This is to send the process to the background (suspend).
  //installSignal(SIGTTIN);
  //installSignal(SIGTTOU);
  //installSignal(SIGSYS);
  //installSignal(SIGXCPU);
  //installSignal(SIGXFSZ);

  foo1();

  return 0;
}
</pre>

<br>

# 빌드시 옵션

빌드시 옵션으로 `-g` 옵션을 줘야 합니다. Eclipe에서 Debug 모드로 빌드할 경우 기본적으로 `-g3` 옵션을 주기 때문에 크게 신경쓸 필요가 없습니다. 

<br>

# 실행 결과

실행 결과는 다음과 같습니다.

~~~
$ ./SegmentationFaultCapture

signal 11 (Segmentation fault), address is 0x10a64 from 0x108c8
[bt]: (1) ./SegmentationFaultCapture() [0x108c8]
[bt]: (2) ./SegmentationFaultCapture() [0x108c8]
[bt]: (3) ./SegmentationFaultCapture() [0x108f8]
[bt]: (4) ./SegmentationFaultCapture() [0x10910]
[bt]: (5) ./SegmentationFaultCapture() [0x10928]
[bt]: (6) ./SegmentationFaultCapture() [0x10940]
[bt]: (7) ./SegmentationFaultCapture() [0x1096c]
[bt]: (8) /lib/arm-linux-gnueabihf/libc.so.6(__libc_start_main+0x114) [0xb6c5f294]
~~~

<br>

# addr2line으로 오류 위치 확인

위 결과창만으로는 아직 정확한 오류 위치를 알 수 없기 때문에 `addr2line` 명령어를 이용해서
결과를 조회합니다.

~~~
$ addr2line -f -e ./SegmentationFaultCapture 0x108c8
_Z5crashv
D:\Workspace\RobotLitoService\SegmentationFaultCapture\Debug/../Main.cpp:73
~~~