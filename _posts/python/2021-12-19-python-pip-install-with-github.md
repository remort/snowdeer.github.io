---
layout: post
title: Git을 이용한 pip3 install 방법
category: Python
tag: [Python, Redis, Celery]
---

# Git을 이용한 pip3 install 방법

여러 프로젝트에서 같은 코드를 공유하기 위한 방법 중 하나입니다. 
공용 라이브러리 코드를 GitHub에 업로드하고 `setup.py` 파일을 설정해서
`pip3 install`에 활용할 수 있습니다.

## 공용 코드 디렉토리 구조

예를 들면, 공용 라이브러리 코드의 프로젝트 구조를 다음과 같이 만듭니다.

```
.
├── README.md
├── main.py
├── setup.py
└── snowdeer_common
    ├── __init__.py
    ├── hello.py
    └── utils
        ├── __init__.py
        └── log.py
```

## setup.py

아래에서 중요한 부분은 `packages` 부분입니다.

<pre class="prettyprint">
from setuptools import setup

setup(name='snowdeer_common',
      version='0.1.0',
      description='snowdeer sample',
      url='https://snowdeer.github.io',
      author='snowdeer',
      author_email='snowdeer0314@gmail.com',
      license='Apache',
      packages=['snowdeer_common',
                'snowdeer_common.utils'
                ],
      zip_safe=False)

</pre>

그리고 git에 업로드하면 됩니다.

## pip3 설치 방법

위 공용 코드를 설치하기 위해서는 해당 환경(ex. venv 환경 등)에서 다음 명령어를 입력합니다.

<pre class="prettyprint">
pip3 install -U git+https://github.com/snowdeer/snowdeer_common
</pre>

