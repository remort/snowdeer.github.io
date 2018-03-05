---
layout: post
title: CentOS에 Java 설치하기
category: Java
tag: [java, centos]
---
# Install Java on CentOS

## 설치 가능 버전 확인

`yum list java*jdk-devel` 명령으로 현재 설치 가능한 Java 패키지 버전을 확인합니다.

<pre class="prettyprint">
$ yum list java*jdk-devel

Loaded plugins: fastestmirror
base                                                     | 3.6 kB     00:00
extras                                                   | 3.4 kB     00:00
updates                                                  | 3.4 kB     00:00
(1/4): extras/7/x86_64/primary_db                          | 166 kB   00:00
(2/4): base/7/x86_64/group_gz                              | 156 kB   00:00
(3/4): updates/7/x86_64/primary_db                         | 6.0 MB   00:00
(4/4): base/7/x86_64/primary_db                                                             | 5.7 MB  00:00:05
Determining fastest mirrors
 * base: ftp.daumkakao.com
 * extras: ftp.daumkakao.com
 * updates: ftp.daumkakao.com
Available Packages
java-1.6.0-openjdk-devel.x86_64                         1:1.6.0.41-1.13.13.1.el7_3                          base
java-1.7.0-openjdk-devel.x86_64                         1:1.7.0.171-2.6.13.0.el7_4                          updates
java-1.8.0-openjdk-devel.i686                           1:1.8.0.161-0.b14.el7_4                             updates
java-1.8.0-openjdk-devel.x86_64                         1:1.8.0.161-0.b14.el7_4                             updates
</pre>

<br>

## 설치

위에서 확인한 패키지 명을 보고 설치를 원하는 Java 패키지를 선택합니다.

<pre class="prettyprint">
sudo yum install -y java-1.8.0-openjdk-devel.x86_64
</pre>

<br>

## 확인

<pre class="prettyprint">
$ java -version

openjdk version "1.8.0_161"
OpenJDK Runtime Environment (build 1.8.0_161-b14)
OpenJDK 64-Bit Server VM (build 25.161-b14, mixed mode)
</pre>