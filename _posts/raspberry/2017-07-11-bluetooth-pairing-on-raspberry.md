---
layout: post
title: 블루투스 페어링하기
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 블루투스]
---

라즈베리 파이(Raspberry PI)와 블루투스 기기간의 페어링하는 방법입니다. 터미널 명령어를 이용한 블루투스 페이렁 방법을 소개합니다.

먼저 라즈베리 파이 터미널에서 다음 명령어를 입력합니다.

~~~
bluetoothctl
~~~

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-11-bluetooth-pairing-on-raspberry/01.png)

위 스크린샷과 같이 프롬프트가 ‘[bluetooth]’로 바뀌면, 이제 블루투스 명령어들을 사용할 수 있습니다.

이 상태에서 사용할 수 있는 블루투스 명령어들은 다음과 같습니다. (명령어 ‘help’를 통해 확인할 수 있습니다.)

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-11-bluetooth-pairing-on-raspberry/02.png)

<br>

라즈베리 파이의 블루투스 정보는 다음 명령어를 이용해 조회할 수 있습니다.

~~~
show
~~~

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-11-bluetooth-pairing-on-raspberry/03.png)

<br>

다른 디바이스들을 탐색하기 위해서 디스커버리(Discovery)를 수행하거나 종료하는 명령어는 다음과 같습니다.
이 때, 페어링하고 싶은 디바이스(ex. 스마트폰)에서도 블루투스 검색 허용을 활성화해줘야
아래 명령어로 검색을 할 수 있습니다. 탐색이 되지 않은 상태에서
Bluetooth MAC Address를 이용한 페어링을 시도했을 때는 페어링이 되지 않더군요.

~~~
scan on
scan off
~~~

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-11-bluetooth-pairing-on-raspberry/04.png)

<br>

자기 자신을 다른 디바이스에서 보이도록 하는 명령어는 다음과 같습니다.

~~~
discoverable on
discoverable off
~~~

<br>

페어링을 원하는 디바이스의 맥 어드레스(MAC Address)를 기억해둡니다.
그리고 페어링을 시도합니다.

~~~
pair [MAC Address]
~~~

<br>

페어링이 되면 스마트폰에 페어링 확인 창이 뜹니다. 스마트폰 화면에서 확인을 눌러줍니다.
그 다음 trust 명령어를 수행합니다.

~~~
trust [MAC Address]
~~~

![image -fullwidth]({{ site.baseurl }}/assets/2017-07-11-bluetooth-pairing-on-raspberry/05.png)

<br>

이제 페어링이 끝났습니다. 다음 명령어를 통해 현재 페어링된 디바이스 목록을 조회할 수 있습니다.

~~~
paired-devices
~~~
