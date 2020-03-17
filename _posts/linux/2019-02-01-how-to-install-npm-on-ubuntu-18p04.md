---
layout: post
title: Ubuntu 18.04에 npm 설치하는 방법(sudo apt install npm 명령어 오류 발생시)

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
## Ubuntu 18.04에 npm 설치하는 방법

`Node.js`는 설치되어 있는데, `npm`이 설치되어 있지 않은 경우 대부분

<pre class="prettyprint">
sudo apt install npm
</pre>

명령어로 `npm` 설치가 가능합니다.

하지만, 만약

~~~
Reading package lists... Done
Building dependency tree       
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 npm : Depends: node-gyp (>= 0.10.9) but it is not going to be installed
E: Unable to correct problems, you have held broken packages.
~~~

오류가 발생할 경우

<pre class="prettyprint">
sudo apt-get install nodejs-dev node-gyp libssl1.0-dev

sudo apt install npm
</pre>

명령어를 이용해서 설치할 수 있습니다.