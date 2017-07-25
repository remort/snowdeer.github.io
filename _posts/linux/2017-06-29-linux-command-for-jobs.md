---
layout: post
title: 작업 관련 명령어들
category: Linux
tag: [Linux 명령어]
---

최근에 []구글 클라우드 플랫폼(GCP, Google Cloud Platform)](https://console.cloud.google.com/)을
이용해서 [폴로닉스(Poloniex)](https://poloniex.com/) 비트코인 랜딩봇을 돌려보고 있습니다.
그런데, 구글 클라우드 플랫폼에 SSH 터미널로 접속해서 랜딩봇을 실행한 후 터미널을 빠져나오면
해당 프로그램이 항상 종료가 되어서 다음 명령어들을 찾아보고 공부해보게 되었습니다.

<br>

## 프로그램 실행

터미널에서 랜딩봇을 실행하는 명령어는 다음과 같습니다.
(다음 번에 터미널에서 복사해서 붙여넣기 편하게 하기 위해서 적어봅니다.)
~~~
python lendingbot.py
~~~

<br>

또는 데몬형태의 백그라운드로 돌릴 수도 있습니다.
~~~
python lendingbot.py &amp;
~~~

<br>
그리고 해당 프로세스가 잘 돌아가는지 확인하는 것은 다음 명령어를 이용해서 확인할 수 있습니다.
~~~
ps -efc | grep python
~~~

<br>

또한 실행중인 프로그램을 잠깐 멈추는 방법은 <kbd>Ctrl</kbd> + <kbd>Z</kbd> 키를 누르면 됩니다. 프로그램은 'Stopped' 상태가 됩니다.
<br>

## Job control commands in Linux
<ul>
 	<li>jobs : 작업 리스트를 출력합니다.</li>
 	<li>bg [job id] : 해당 id의 작업을 백그라운드로 보냅니다. '&amp;'를 이용해서 프로그램을 실행한 거랑 동일한 효과입니다. 백그라운드로 보내면서 프로그램은 'Running' 상태가 됩니다.</li>
 	<li>fg [job id] : 해당 id의 작업을 포어그라운드로 가져옵니다.</li>
 	<li>kill [job id] : 해당 id의 작업을 정리합니다.</li>
 	<li>disown [job id] : 해당 id의 작업의 소유권을 지워버립니다. 원래는 해당 작업의 소유권을 터미널이 갖게 되는데 이 명령어를 이용하면 해당 소유권이 지워지고 터미널을 로그아웃해도 프로그램은 계속 동작하게 됩니다. disown 뒤 작업 id가 오류를 내기도 하는데, 그런 경우는 disown 만 단독으로 사용할 수도 있습니다.</li>
 	<li>nohup [명령어] : 터미널에서 종료를 해도 명령어 부분이 계속 실행되도록 하는 명령어
ex) nohup python lendingbot.py &amp;</li>
</ul>
