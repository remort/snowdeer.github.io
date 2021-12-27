---
layout: post
title: Python Log Class
category: Python
tag: [Python, Redis, Celery]
---

# Python Log Class

Python에는 기본적으로 훌륭한 Log 클래스들이 존재하지만, 커스트마이즈를 쉽게 하기 위해서는
간단한 Log 클래스 하나 정도는 갖고 있는게 좋을 듯 합니다.

<pre class="prettyprint">
import threading
from datetime import datetime


class Log:
    __BLACK = "\033[30m"
    __RED = "\033[31m"
    __GREEN = "\033[32m"
    __YELLOW = "\033[33m"
    __BLUE = "\033[34m"
    __MAGENTA = "\033[35m"
    __CYAN = "\033[36m"
    __WHITE = "\033[37m"
    __UNDERLINE = "\033[4m"
    __RESET = "\033[0m"

    __lock = threading.Lock()

    @classmethod
    def d(cls, message):
        Log.__print_log(Log.__GREEN, message)

    @classmethod
    def i(cls, message):
        Log.__print_log(Log.__YELLOW, message)

    @classmethod
    def w(cls, message):
        Log.__print_log(Log.__MAGENTA, message)

    @classmethod
    def e(cls, message):
        Log.__print_log(Log.__RED, message)

    @classmethod
    def __print_log(cls, color: int, message: str):
        with Log.__lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            print(f"{color}[{timestamp}] {message}{Log.__RESET}")

</pre>
