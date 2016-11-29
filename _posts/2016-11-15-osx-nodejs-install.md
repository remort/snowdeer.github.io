---
layout: post
title: OSX에서 Node.js 설치하기
category: Node.js
tag: [osx, Node.js]
---

OSX에서 Node.js를 설치해보도록 하겠습니다.
여러가지 방법이 있습니다.

* 직접 [Node.js 공식 사이트](https://nodejs.org/en/download/)에서 다운받아서 설치
* [홈브류](http://brew.sh/index_ko.html)를 이용한 설치
* [Node Version Manager](https://github.com/creationix/nvm)를 이용한 방법

직접 공식 사이트에서 Node.js 설치 파일을 다운받아서 설치하는 방법이 가장 간편하지만 
관리자 권한과 관련한 귀찮은 문제가 있고, 또한 여러 버전의 Node.js 패키지를 관리하기가 어렵습니다.

홈브류를 이용한 설치 또한 종종 충돌 현상이 발생한다고 합니다.

저는 nvm이라고 불리우는 Node Version Manager를 이용하여 OSX에서 Node.js를 설치해보도록 하겠습니다.
상세한 설명은 [여기](https://github.com/creationix/nvm)로 가시면 보실 수 있습니다.

<br>

## nvm 설치

터미널을 열어서 다음 명령어를 통해 nvm을 설치합니다.

<pre class="prettyprint" style="font-size:0.7em;">
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.32.1/install.sh | bash
</pre>

![image]({{ site.baseurl }}/assets/2016-11-15-osx-nodejs-install/1.png)

위 이미지와 같은 화면이 나오면 설치가 완료된 것입니다.
다만, 아래쪽에

~~~
=> Profile not found. Tried  (as defined in $PROFILE), ~/.bashrc, ~/.bash_profile, ~/.zshrc, and ~/.profile.
=> Create one of them and run this script again
=> Create it (touch ) and run this script again
   OR
=> Append the following lines to the correct file yourself:

export NVM_DIR="/Users/snowdeer/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
=> Close and reopen your terminal to start using nvm or run the following to use it now:

export NVM_DIR="/Users/snowdeer/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
~~~

라는 문구가 있는데, 터미널의 환경 변수를 위한 profile 파일이 없어서 발생하는 문제입니다.

<br>

## 환경 변수 설정

기존에 만들어놓은 profile 파일이 있을 경우는 큰 문제가 없는데, 한 번도 만들어놓지 않은 경우는
다음과 같이 profile 파일을 직접 만들면 됩니다.

저는 .bash_profile 파일을 만들도록 하겠습니다.

터미널에서

<pre class="prettyprint" style="font-size:0.7em;">
touch .bash_profile
</pre>

를 입력합니다.

그리고 'ls -a' 명령어를 통해 .bash_profile 파일이 잘 만들어졌는지 확인합니다.

이후, 터미널에서

<pre class="prettyprint" style="font-size:0.7em;">
open -e .bash_profile
</pre>

명령어를 이용해서 .bash_profile 파일을 에디터기에서 열도록 합시다.
(GUI 에디터이기 때문에 겁먹을 필요 없습니다.)

여기에 다음과 같이 nvm 설치시 나왔던 환경 변수 코드를 입력해줍니다.

<pre class="prettyprint" style="font-size:0.7em;">
export NVM_DIR="/Users/snowdeer/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"  # This loads nvm
</pre>

![image]({{ site.baseurl }}/assets/2016-11-15-osx-nodejs-install/2.png)

그리고 터미널을 재시작합니다.

nvm 및 환경 변수가 잘 설정되었는지 확인하기 위해서 터미널에서 'nvm'을 입력해봅니다.

![image]({{ site.baseurl }}/assets/2016-11-15-osx-nodejs-install/3.png)


<br>

## Node.js 설치

이제 nvm을 이용해서 Node.js를 설치할 차례입니다.

nvm에 대한 사용법은 GitHub의 nvm 페이지의 Usage 항목을 보시는게 편리합니다.
[여기](https://github.com/creationix/nvm)를 통해 해당 페이지로 가실 수 있습니다.

간단하게 설명드리면,

<pre class="prettyprint" style="font-size:0.7em;">
nvm install node
</pre>

을 통해 Node.js를 설치할 수 있습니다.
특정 버전의 설치를 원할 경우

<pre class="prettyprint" style="font-size:0.7em;">
nvm install v7.2.0
</pre>

와 같이 설치할 수도 있으며, 안정적인 stable 버전의 설치를 원할 경우

<pre class="prettyprint" style="font-size:0.7em;">
nvm install stable
</pre>

명령어를 내릴 수도 있습니다.
설치할 수 있는 버전 리스트는

<pre class="prettyprint" style="font-size:0.7em;">
nvm ls-remote
</pre>

명령어를 통해 확인할 수 있습니다.

만약 여러 버전의 Node.js를 설치한 경우는

<pre class="prettyprint" style="font-size:0.7em;">
nvm use v4.2.6
</pre>

명령어를 통해 해당 버전의 Node.js로 사용 선택할 수 있습니다.


그냥 stable로 설치를 해보도록 하겠습니다. 

![image]({{ site.baseurl }}/assets/2016-11-15-osx-nodejs-install/4.png)

위 그림과 같은 결과가 나오면 설치 완료입니다.


