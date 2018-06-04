---
layout: post
title: Web SSH Server 설치 방법
category: Linux
tag: [Linux]
---
# Web 브라우저에서 접속할 수 있는 SSH 서버 설치

`shellinabox`라는 프로그램을 통해서 웹 브라우저에서 접속할 수 있는 SSH 서버를 설치할 수 있습니다.

<br>

## shellinabox 설치

~~~
sudo apt-get install shellinabox
~~~

<br>

## 포트 변경 등

~~~
sudo nano /etc/default/shellinabox
~~~

<br>

## 서비스 실행/중지

~~~
sudo service shellinabox stop
~~~

<br>

## 상태 조회

~~~
sudo netstat -nap | grep shellinabox
~~~

<br>

## css 적용

만약 css를 적용하고 싶으면 먼저 서비스를 중지시킨다음 터미널에서 다음과 같이 입력하면 됩니다.

~~~
shellinaboxd --css "/home/snowdeer/white-on-black.css"
~~~

또는 아예 `config` 파일에 설정하면 컴퓨터를 재실행해도 항상 적용이 되기 때문에 더 편리합니다.

~~~
sudo nano /etc/default/shellinabox

...

SHELLINABOX_ARGS="--no-beep --css /home/snowdeer/white-on-black.css"
~~~

<br>

## white-on-black.css 파일

`shellinabox` 기본값은 하얀 바탕에 검은 글씨인데, 만약 반대 색상을 사용하고 싶으면 아래 `css` 파일을 적용해서 테마를 변경할 수 있습니다.

~~~
#vt100 #cursor.bright {
  background-color: white;
  color:            black;
}

#vt100 #cursor.dim {
  background-color: black;
  opacity:          0.2;
  -moz-opacity:     0.2;
  filter:           alpha(opacity=20);
}

#vt100 #scrollable {
  color:            #ffffff;
  background-color: #000000;
}

#vt100 #scrollable.inverted {
  color:            #000000;
  background-color: #ffffff;
}

#vt100 .ansiDef {
  color:            #ffffff;
}

#vt100 .ansiDefR {
  color:            #000000;
}

#vt100 .bgAnsiDef {
  background-color: #000000;
}

#vt100 .bgAnsiDefR {
  background-color: #ffffff;
}

#vt100 #scrollable.inverted .ansiDef {
  color:            #000000;
}

#vt100 #scrollable.inverted .ansiDefR {
  color:            #ffffff;
}

#vt100 #scrollable.inverted .bgAnsiDef {
  background-color: #ffffff;
}

#vt100 #scrollable.inverted .bgAnsiDefR {
  background-color: #000000;
}
~~~