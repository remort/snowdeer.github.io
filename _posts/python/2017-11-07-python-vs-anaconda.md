---
layout: post
title: Python vs 아니콘다(Anaconda) 차이점
category: Python
tag: [Python]
---

Python과 아나콘다(Anaconda)의 차이점은 다음과 같습니다.

![Image](/assets/2017-11-07-python-vs-anaconda/01.png)

Python은 [파이썬 공식 홈페이지](https://www.python.org/)에서 받을 수 있으며, `pip` 툴만을 포함하고 있습니다. 필요한 패키지나 라이브러리 등을 설치할 때 모두 수동으로 해줘야 합니다.

아나콘다는 Python 기본 패키지에 각종 수학/과학 라이브러리들을 같이 패키징해서 배포하는 버전입니다. [여기](https://www.anaconda.com/download/)에서 다운로드 할 수 있습니다. 아나콘다에 포함된 툴들로는 대표적으로 `panda`, `numpy`, `scipy`, `sklearn`, `matplotlib`, `Jupyter Notebook` 등이 있습니다.

<br>

# 설치시 유의 사항

Python이나 아나콘다 중 하나만 설치하는 것을 추천합니다. 둘 다 설치할 경우 중복되는 파일들이 많으며 환경 변수 충돌 등의 문제가 발생할 수도 있습니다.

<br>

# 어떤 것을 선택할 것인가

아나콘다에 포함되어 있는 라이브러리들이 불필요한 경우에는 기본 Python만 설치해도 무관합니다. 하지만, 요즘 유행하는 인공지능이나 빅데이터 관련 개발을 할 경우에는 결국 아나콘다에 포함된 라이브러리들을 설치할 가능성이 높기 때문에 애초에 아나콘다를 설치하는 것이 더 유리합니다. 일일이 라이브러리들을 설치하다보면 의존성 문제 등이 발생할 수도 있기 때문입니다.

<br>

# 아나콘다 사용법

~~~
conda search python
~~~

설치되어 있는 Python의 버전 또는 라이브러리들의 버전을 확인할 수 있습니다.

<br>

~~~
conda create -n py36 python=3.6.1 anaconda
~~~

3.6.1 버전의 Python을 사용하는 `py36`이라는 환경을 생성합니다. 생성된 환경은 아래의 명령어를 이용해서 활성화/비활성화를 할 수 있습니다.

~~~
active py36
deactive py36
~~~

<br>

~~~
conda install python=3.6.1
~~~

위 명령어는 아나콘다의 Python 기본 버전을 설정하는 명령어입니다.