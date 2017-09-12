---
layout: post
title: Ubuntu에 JDK 설치하는 방법
category: Linux
tag: [리눅스 명령어, JDK]
---
# Ubuntu에 JDK 설치하는 방법

## 오라클 공식 사이트에서 다운받아 설치

[오라클 공식 배포 사이트](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html)에서 JDK 설치 파일을 다운받아 설치하는 방법이 있습니다. 가장 확실한 방법 중의 하나입니다.

다만, 여기에서 내려 받는 파일은 확장자가 `rpm`으로 되어 있습니다. 이 경우, `deb` 확장자로 변경해주어야 설치를 할 수 있습니다.

[`rpm` 파일을 `deb` 파일로 변경](linux/2016/03/11/install-deb-and-rpm-package)하고 나면 다음 명령어를 이용해서 JDK를 설치할 수 있습니다.

~~~
$ sudo dpkg -i jdk1.8.0-144_1.8.014401_amd64.deb
~~~

<br>

## 확장자 변환하지 않고 직접 설치

오라클 공식 사이트에서 `tar.gz` 형태의 파일을 내려 받은 뒤 직접 설치하는 방법입니다.

먼저 다음 명령어를 이용해서 압축을 해제하고 파일들을 `/usr/lib/jvm` 폴더 아래로 이동시켜줍니다.

~~~
$ tar zxvf jdk-8u144-linux-x64.tar.gz
$ sudo mkdir /usr/lib/jvm
$ sudo mv jdk1.8.0_144 /usr/lib/jvm
~~~

그리고 다음 명령어를 통해 명령어 등록을 해 줍니다.

~~~
$ sudo update-alternatives --install /usr/bin/java java /usr/lib/jvm/jdk1.8.0_144/bin/java 1
$ sudo update-alternatives --install /usr/bin/javac javac /usr/lib/jvm/jdk1.8.0_144/bin/javac 1
$ sudo update-alternatives --install /usr/bin/javaws javaws /usr/lib/jvm/jdk1.8.0_144/bin/javaws 1
$ sudo update-alternatives --config java
$ sudo update-alternatives --config javac
$ sudo update-alternatives --config javaws
~~~


<br>

## PPA를 이용한 방법

PPA(Personal Package Archive)에서 JDK를 내려 받아 설치하는 방법입니다. `apt-get` 명령어를 이용한 설치라서 간편합니다.

~~~
$ sudo apt-add-repository ppa:webupd8team/java
$ sudo apt-get update
$ sudo apt-get install oracle-java8-installer
~~~
