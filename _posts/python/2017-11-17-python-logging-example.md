---
layout: post
title: Logging 모듈 사용 방법
category: Python
tag: [Python]
---
# logging 모듈

Python에서는 로그 출력을 위한 `logging` 모듈을 제공합니다. 아주 간단히 사용할 수 있으며, `print` 함수 등을 통해 콘솔창에 지저분하게 출력하는 것보다 `logging` 모듈을 사용하는 것을 추천합니다.

<br>

## 로그 레벨

Python에서 로그 레벨은 다음과 같습니다. 안드로이드 등에서와 거의 동일합니다.

* DEBUG
* INFO
* WARNING
* ERROR
* CRITICAL

<br>

## 간단한 logging 모듈 사용 예제

<pre class="prettyprint">
import logging as log

log.basicConfig(filename='./log.txt', level=log.DEBUG)

if __name__ == '__main__':
    log.debug('debug')
    log.info('info')
    log.warning('warning')
    log.error('error')
    log.critical('critical')
</pre>

출력 결과는 다음과 같습니다.

~~~
DEBUG:root:debug
INFO:root:info
WARNING:root:warning
ERROR:root:error
CRITICAL:root:critical
~~~

`basicConfig` 함수를 통해서 로그 파일의 이름을 지정할 수 있고, `filemode` 속성의 값을 `a`, `w` 등으로 지정할 수 있습니다. `a`의 경우 기존 파일에 이어서 기록(Append)하며, `w`의 경우 기존 파일 내용을 삭제하고 새로 작성합니다.

<br>

## 콘솔창과 파일에 동시에 로그 남기기

<pre class="prettyprint">
import logging
import logging.handlers

log = logging.getLogger('snowdeer_log')
log.setLevel(logging.DEBUG)

fileHandler = logging.FileHandler('./log.txt')
streamHandler = logging.StreamHandler()

log.addHandler(fileHandler)
log.addHandler(streamHandler)

if __name__ == '__main__':
    log.debug('debug')
    log.info('info')
    log.warning('warning')
    log.error('error')
    log.critical('critical')
</pre>

<br>

## Formatter 적용하기

<pre class="prettyprint">
import logging
import logging.handlers

log = logging.getLogger('snowdeer_log')
log.setLevel(logging.DEBUG)

formatter = logging.Formatter('[%(levelname)s] (%(filename)s:%(lineno)d) > %(message)s')

fileHandler = logging.FileHandler('./log.txt')
streamHandler = logging.StreamHandler()

fileHandler.setFormatter(formatter)
streamHandler.setFormatter(formatter)

log.addHandler(fileHandler)
log.addHandler(streamHandler)

if __name__ == '__main__':
    log.debug('debug')
    log.info('info')
    log.warning('warning')
    log.error('error')
    log.critical('critical')
</pre>

출력 결과는 다음과 같습니다.

~~~
[DEBUG] (log_test.py:19) > debug
[INFO] (log_test.py:20) > info
[WARNING] (log_test.py:21) > warning
[ERROR] (log_test.py:22) > error
[CRITICAL] (log_test.py:23) > critical
~~~

<br>

### Formatter Keywords

이름 | 포맷 | 설명
---|---|---
asctime | %(asctime)s | 날짜 시간, 밀리세컨드까지 출력. ex) 2017.11.17 12:31:45,342
created | %(created)f | 생성 시간 출력
filename | %(filename)s | 파일명
funcname | %(funcname)s | 함수명
levelname | %(levelname)s | 로그 레벨
lineno | %(lineno)d | 소스의 라인 넘버
module | %(module)s | 모듈 이름
msecs | %(msecs)d | 로그 생성 시간에서 밀리세컨드 시간 부분만 출력
message | %(message)s | 로그 메시지
name | %(name)s | 로그 이름
pathname | %(pathname)s | 소스 경로
process | %(process)d | 프로세스(Process) Id
processName | %(processName)s | 프로세스 이름
thread | %(thread)d | Thread Id
threadName | %(threadName)s | Thread Name

<br>

## 파일의 최대 용량 및 새로운 파일 생성하는 방법

`RotatingFileHandler` 함수를 이용해서 파일의 최대 크기 및 새로운 파일을 생성할 때 최대 개수를 지정해줄 수 있습니다. 아래와 같은 코드를 이용해서 최대 10MB씩, 총 20개의 로그 파일을 만들어서 순환하도록 할 수 있습니다.

<pre class="prettyprint">
log_max_size = 10 * 1024 * 1024
log_file_count = 20
fileHandler = logging.handlers.RotatingFileHandler(filename='./log.txt', maxBytes=log_max_size,
                                                   backupCount=log_file_count)
</pre>

<br>

## logging.handlers

위에서는 `FileHandler`와 `StreamingHandler` 및 `RotatingFileHandler` 등을 사용했었는데 이외에도 다양한 핸들러들이 존재합니다.

핸들러 | 설명
---|---
SocketHandler | 외부 로그 서버로 소켓을 통해 전송
DatagramHandler | UDP 통신을 통해 외부 서버로 전송
SysLogHandler | Unix 류의 syslog 데몬에게 로그 전송
SMTPHandler | 메일로 로그 전송
HTTPHandler | HTTP를 통해 로그 전송