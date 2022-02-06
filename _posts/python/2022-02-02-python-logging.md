---
layout: post
title: Python logging 예제
category: Python
tag: [Python]
---

# Pythong logging 모듈 사용하기

Python에 기본으로 내장된 `logging` 모듈에 대한 사용법입니다.
로그 레벨에 따라 `debug`, `info`, `warning`, `error`, `critical`의 5가지 등급이 있으며 
이 중에서 `warning`는 `warn`이라는 메소드도 같이 존재합니다. `warn`은 `deprecated` 되었기 때문에,
가급적 `warn`은 사용하지 말고 `warning`을 사용하도록 주의합시다.

## 간단한 예제

<pre class="prettyprint">
import logging


def main():
    logging.debug('debug')
    logging.info('info')
    logging.warning('warning')
    logging.error('error')
    logging.critical('critical')


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG,
                        format='[%(asctime)s] [%(levelname)s] %(message)s (%(filename)s:%(lineno)d)')
    main()
</pre>

<br>

# Colored logging 사용하기

여기에 색상을 입히는 코드를 적용해봅니다.

## color_palette.py

<pre class="prettyprint">
# Reset
RESET = '\033[0m'

# Regular Colors
BLACK = '\033[30m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
MAGENTA = '\033[35m'
CYAN = '\033[36m'
WHITE = '\033[37m'

# Bold
BOLD_BLACK = '\033[1;30m'
BOLD_RED = '\033[1;31m'
BOLD_GREEN = '\033[1;32m'
BOLD_YELLOW = '\033[1;33m'
BOLD_BLUE = '\033[1;34m'
BOLD_MAGENTA = '\033[1;35m'
BOLD_CYAN = '\033[1;36m'
BOLD_WHITE = '\033[1;37m'

# Underline
UNDERLINE_BLACK = '\033[4;30m'
UNDERLINE_RED = '\033[4;31m'
UNDERLINE_GREEN = '\033[4;32m'
UNDERLINE_YELLOW = '\033[4;33m'
UNDERLINE_BLUE = '\033[4;34m'
UNDERLINE_MAGENTA = '\033[4;35m'
UNDERLINE_CYAN = '\033[4;36m'
UNDERLINE_WHITE = '\033[4;37m'

# High Intensity
INTENSITY_BLACK = '\033[0;90m'
INTENSITY_RED = '\033[0;91m'
INTENSITY_GREEN = '\033[0;92m'
INTENSITY_YELLOW = '\033[0;93m'
INTENSITY_BLUE = '\033[0;94m'
INTENSITY_MAGENTA = '\033[0;95m'
INTENSITY_CYAN = '\033[0;96m'
INTENSITY_WHITE = '\033[0;97m'

</pre>

## colored_log_handler.py

<pre class="prettyprint">
import logging

from color_palette import RESET, GREEN, WHITE, YELLOW, MAGENTA, RED


class ColoredLogHandler(logging.StreamHandler):
    def __init__(self):
        super().__init__()
        self.setLevel(logging.DEBUG)
        self.setFormatter(self.__LogFormatter())

    class __LogFormatter(logging.Formatter):
        __FORMAT_DEBUG = '[%(asctime)s] [%(levelname)s] %(message)s (%(filename)s:%(lineno)d)'
        __FORMAT_INFO = '[%(asctime)s] [%(levelname)s] %(message)s'
        __FORMAT_WARNING = '[%(asctime)s] [%(levelname)s] %(message)s'
        __FORMAT_ERROR = '[%(asctime)s] [%(levelname)s] %(message)s'
        __FORMAT_CRITICAL = '[%(asctime)s] [%(levelname)s] %(message)s (%(filename)s:%(lineno)d)'

        FORMATS = {
            logging.DEBUG: GREEN + __FORMAT_DEBUG + RESET,
            logging.INFO: WHITE + __FORMAT_INFO + RESET,
            logging.WARNING: YELLOW + __FORMAT_WARNING + RESET,
            logging.ERROR: MAGENTA + __FORMAT_ERROR + RESET,
            logging.CRITICAL: RED + __FORMAT_CRITICAL + RESET
        }

        def format(self, record):
            log_fmt = self.FORMATS.get(record.levelno)
            formatter = logging.Formatter(log_fmt)
            return formatter.format(record)

</pre>

## main.py

<pre class="prettyprint">
import logging
from colored_log_handler import ColoredLogHandler


def main():
    logging.debug('debug')
    logging.info('info')
    logging.warning('warning')
    logging.error('error')
    logging.critical('critical')


if __name__ == '__main__':
    logging.basicConfig(level="DEBUG", handlers=[ColoredLogHandler()])

    main()

</pre>