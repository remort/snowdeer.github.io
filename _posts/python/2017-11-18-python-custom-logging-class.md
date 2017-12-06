---
layout: post
title: Custom Logging 클래스 예제
category: Python
tag: [Python]
---
# Custom Logging 클래스

리눅스에서 사용하기 위한 Custom Logging 클래스를 간단하게 작성해보았습니다. 안드로이드에서의 로그 메소드들과 비슷하게 사용할 수 있도록 변경했습니다.

## log.py

<pre class="prettyprint">
import logging
import logging.handlers

class Log:
    __logger = logging.getLogger('SnowLog')
    __logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s > [%(levelname)s] %(message)s')
    fileHandler = logging.FileHandler('./log.txt')
    fileHandler.setFormatter(formatter)

    __logger.addHandler(fileHandler)

    @classmethod
    def d(cls, tag, message):
        cls.__logger.debug('[' + tag + '] ' + message)

    @classmethod
    def i(cls, tag, message):
        cls.__logger.info('[' + tag + '] ' + message)

    @classmethod
    def w(cls, tag, message):
        cls.logger.warning('[' + tag + '] ' + message)

    @classmethod
    def e(cls, tag, message):
        cls.logger.error('[' + tag + '] ' + message)

    @classmethod
    def c(cls, tag, message):
        cls.logger.critical('[' + tag + '] ' + message)
</pre>

<br>

## 리눅스에서 로그 확인 할 때

사실 로그를 콘솔창이 아닌 파일로 출력하도록 한 건 로그 백업의 목적도 있지만, 파이썬의 경우 콘솔창으로 출력을 하게 될 경우 콘솔에서의 입력인 `input()` 메소드와 매끄럽게 동작하지 않아서였습니다. 단일 Thread의 경우는 문제가 없지만 복수의 Thread에서 로그를 출력할 경우 출력시마다 `input()` 버퍼의 내용을 지워버리는 문제가 있었습니다.

그래서 파일로 로그를 남길 경우 새로운 터미널을 하나 열고 다음 명령어를 입력해서 로그를 실시간으로 확인할 수 있습니다.

~~~
tail -f [파일명]

ex) tail -f log.txt
~~~
