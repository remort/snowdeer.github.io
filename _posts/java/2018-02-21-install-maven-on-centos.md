---
layout: post
title: CentOS에 Maven 설치하기
category: Java
tag: [java, maven, centos]
---
# Install Maven on CentOS

Maven을 설치하기 전에 먼저 Java가 설치되어 있어야 합니다.

<br>

## Maven 다운로드

Maven 다운로드는 [여기](https://maven.apache.org/download.cgi)에서 할 수 있습니다.

<pre class="prettyprint">
$ wget http://apache.tt.co.kr/maven/maven-3/3.5.2/binaries/apache-maven-3.5.2-bin.tar.gz
</pre>

<br>

## 압축해제 및 파일 이동, 링크 생성

<pre class="prettyprint">
tar -xvzf apache-maven-3.5.2-bin.tar.gz
sudo mv apache-maven-3.5.2 /usr/local/maven
sudo ln -s /usr/local/maven/bin/mvn /usr/bin/mvn
</pre>

<br>

## 환경 변수 추가

<pre class="prettyprint">
sudo su
cat<<EOF> /etc/profile.d/maven.sh
#!/bin/bash 
MAVEN_HOME=/srv/maven 
PATH=$MAVEN_HOME/bin:$PATH 
export PATH MAVEN_HOME 
export CLASSPATH=.
EOF
exit
</pre>

<br>

## 설치 확인

<pre class="prettyprint">
mvn -version
</pre>