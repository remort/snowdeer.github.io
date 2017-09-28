---
layout: post
title: Linux의 저수준 파일 처리
category: C++
tag: [C++, Linux, File]
---
# 파일 처리

리눅스에서는 디렉토리 및 디바이스들도 모두 파일로 취급됩니다. 리눅스는 POSIX 기반으로 파일에 대한 시스템 콜(System Call)을 처리합니다.

<br>

## 저수준 파일 입출력 명령어

리눅스에서 제공하는 POSIX 기반 파일 시스템 제어 함수들은 다음과 같습니다. 저수준 파일 제어 함수들은 `<unistd.h>`에 선언되어 있습니다.

함수 | 설명
--- | ---
open() | 파일을 읽기/쓰기 등을 위해 파일을 열거나 생성
creat() | 파일을 생성한다. open()으로 대체 가능
close() | 열려있는 파일을 닫는다.
read() | 파일로부터 데이터를 읽는다.
write() | 파일에 데이터를 기록
lseek() | 파일 포인터를 특정 위치로 이동
unlink() | 파일을 삭제
remove() | 파일이나 디렉토리를 삭제
fcntl() | 파일 속성을 설정하거나 조정함. ioctl()로 대체 가능
dup() | 파일 디스크립터(descriptor)를 복사. dup2()로 대체 가능

<br>

## open()

파일을 읽거나 쓰기 위해서는 먼저 파일을 열어야 합니다.

<pre class="prettyprint">
int open(const char *path, int flags, ...);
</pre>

파일을 열 떄 `flags`로는 다음과 같은 값들을 사용할 수 있습니다.

Flag | 설명
---|---
O_RDONLY | 읽기 전용
O_WRONLY | 쓰기 전용
O_RDWR | 읽기/쓰기 모두 가능
O_APPEND | 쓰기 작업시 파일 끝에 내용 추가
O_CREAT | 파일이 없을 경우 생성
O_EXCL | 파일이 존재할 경우 에러 발생
O_TRUNC | 파일이 존재할 경우 삭제
O_NONBLOCK | Non-Blocking 모드로 전환
O_SYNC | 쓰기 연산마다 버퍼(Buffer)없이 디스크에 바로 저장

<br>

## creat()

새로운 파일을 생성하는 함수입니다. 보통 `open()` 함수로 대체해서 쓸 수 있기 때문에 그리 많이 사용되지는 않습니다.

<pre class="prettyprint">
int creat(const char *path, mode_t mode);
</pre>

<br>

## close()

읽기/쓰기 등의 작업을 끝낸 후에는 반드시 파일을 닫아줘야 합니다.

<pre class="prettyprint">
int close(int fd);
</pre>

<br>

## read()

파일로부터 데이터를 읽어들이며, 데이터를 읽은만큼 offset를 증가시킵니다.

<pre class="prettyprint">
ssize_t read(int fd, void *buf, size_t count);
</pre>

보통 읽어들인 데이터의 바이트 수를 리턴하지만, 파일의 끝(EOF, End of file)을 만나면 `0`을 리턴하고, 읽기에 실패하면 `-1`을 리턴합니다.

<br>

## write()

파일에 데이터를 쓰며, 데이터를 쓴만큼 offset를 증가시킵니다.

<pre class="prettyprint">
ssize_t write(int fd, const void *buf, size_t count);
</pre>

기록한 데이터의 크기만큼 리턴하며, 쓰기 실패시 `-1`을 리턴합니다.

<br>

## lseek()

현재 열려 있는 파일의 원하는 위치로 offset을 이동시킵니다.

<pre class="prettyprint">
off_t lseek(int fd, off_t offset, int whence);
</pre>

마지막 인자인 `whence`에는 기준점을 넣습니다. 기준점으로는 다음과 같은 값을 사용할 수 있습니다.

인자 | 설명
---|---
SEEK_SET | 파일의 처음부터 상대적인 거리
SEEK_CUR | 파일의 현재 offset로부터 상대적인 거리
SEEK_END | 파일의 끝에서부터 상대적인 거리

`lseek()` 함수가 성공적으로 실행되면 해당 위치의 offset를 리턴하며, 실패할 경우에는 `-1`을 리턴합니다.

<br>

## ioctl() / fcntl()

`ioctl()` 함수는 일반 파일, 네트워크 통신 모듈 등의 디바이스와 관련된 속성 연산을 위해 만들어졌습니다.

<pre class="prettyprint">
int ioctl(int fd, int cmd, ...);
</pre>

일반적으로 `ioctl()` 함수가 성공적으로 실행되면 `0` 또는 `0` 보다 큰 수가 리턴되고 실패할 경우 `-1`을 리턴합니다.

`fcntl()` 함수는 일반 파일의 연산을 위해 만들어졌는데, 이미 열려 있는 파일의 속성을 변경할 수 있습니다. 마찬가지로 실패시 `-1`을 리턴합니다.

<pre class="prettyprint">
int fcntl(int fd, int cmd, ...);
</pre>

`fcntl()` 함수의 옵션들은 다음과 같습니다.

옵션 | 설명
---|---
F_DUPFD | 파일 디스크립터를 복사할 때 사용되며, 세 번째 인수보다 크거나 같은 값 중에 가장 작은 미사용 값을 반환
F_GETFD | 파일 디스크립터의 Flag를 리턴
F_SETFD | 파일 디스크립터의 Flag를 설정
F_GETFL | 파일 테이블에 저장되어 있는 파일 상태 Flag(O_APPEND, O_NONBLOCK, O_SYNC 등)를 리턴
F_SETFL | 파일 상태 Flag를 설정
F_GETOWN | SIGSO, SIGURG 시그널을 받는 프로세스 ID 및 프로세스 그룹 ID를 리턴
F_SETOWN | SIGSO, SIGURG 시그널을 받는 프로세스 ID 및 프로세스 그룹 ID를 설정

복수의 프로세스들이 같은 파일에 접근할 때 다른 프로세스들과의 충돌 방지를 위해 파일을 잠궈야 하는 경우가 있습니다. `fcntl()` 함수를 이용해서 이러한 작업이 가능하며, 다른 프로세스들은 파일이 잠긴 동안 대기하도록 할 수 있습니다. `fcntl()` 함수를 이용해서 파일을 잠그는 경우는 3번째 인자로 `flock` 구조체를 사용합니다.
