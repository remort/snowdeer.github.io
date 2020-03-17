---
layout: post
title: ZSH 및 Oh! My ZSH 설치하는 방법
category: Linux
tag: [리눅스]
---
# ZSH 및 Oh! My ZSH 설치하는 방법

<br>

## terminator 설치(생략 가능)

테마 적용도 하기 쉬운 편이며, ZSH와 잘 어울리는 조합이기 때문에 가급적 `terminator`를 설치하는 것을 추천합니다. 

<pre class="prettyprint">
sudo apt-get update
sudo apt-get install terminator
</pre>

<br>

## zsh 설치

<pre class="prettyprint">
sudo apt-get install zsh
</pre>

<br>

## Oh My ZSH 설치

<pre class="prettyprint">
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
</pre>

<br>

## Powerline 폰트 설치(생략 가능)

저는 개인적으로 `D2Coding Regular`를 사용하기 때문에 이 과정은 건너뛰는 편입니다.

<pre class="prettyprint">
wget https://github.com/powerline/powerline/raw/develop/font/PowerlineSymbols.otf
wget https://github.com/powerline/powerline/raw/develop/font/10-powerline-symbols.conf

mkdir ~/.fonts/
mv PowerlineSymbols.otf ~/.fonts/
mkdir -p ~/.config/fontconfig/conf.d 

fc-cache -vf ~/.fonts/
mv 10-powerline-symbols.conf ~/.config/fontconfig/conf.d/
</pre>