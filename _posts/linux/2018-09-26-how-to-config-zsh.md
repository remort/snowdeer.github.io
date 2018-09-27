---
layout: post
title: ZSH 기본 설정 변경
category: Linux
tag: [리눅스]
---
# ZSH 기본 설정 변경

`ZSH` 기본 설정은 `~/.zshrc` 파일에서 세팅할 수 있습니다. `~/.zshrc` 파일은 터미널을 처음 실행했을 때 실행되는 환경 변수 파일이기도 합니다. (`.bashrc` 파일과 유사)

<br>

## 기본 테마를 agnoster로 변경

`~/.zshrc` 파일을 열어서 다음과 같이 변경합니다. 기본 값은 `ZSH_THEME="robbyrussell"` 입니다.

<pre class="prettyprint">
ZSH_THEME="agnoster"
</pre>

만약 실행할 때마다 랜덤으로 테마를 변경하고 싶으면 다음과 같이 설정할 수도 있습니다. 

<pre class="prettyprint">
ZSH_THEME="random"
</pre>

<br>

## 테마 색상(팔레트) 변경

가독성이 좋은 `Solarize` 색상 테마로 변경하는 방법입니다. 저는 `Solarize` 색상 테마 안의 `Dark Theme`를 사용하고 있습니다.

<pre class="prettyprint">
sudo apt-get install dconf-cli

git clone git://github.com/sigurdga/gnome-terminal-colors-solarized.git ~/.solarized
cd ~/.solarized
./install.sh

This script will ask you which color scheme you want, and which Gnome Terminal profile to overwrite.

Please note that there is no uninstall option yet. If you do not wish to overwrite any of your profiles, you should create a new profile before you run this script. However, you can reset your colors to the Gnome default, by running:

    Gnome >= 3.8 dconf reset -f /org/gnome/terminal/legacy/profiles:/
    Gnome < 3.8 gconftool-2 --recursive-unset /apps/gnome-terminal

By default, it runs in the interactive mode, but it also can be run non-interactively, just feed it with the necessary options, see 'install.sh --help' for details.

Please select a color scheme:
1) dark
2) dark_alternative
3) light
#? 1

Please select a Gnome Terminal profile:
1) Unnamed
#? 1
~~~
</pre>

그런 다음 `.zshrc` 파일 맨 아래에 아래 항목을 추가합니다.

<pre class="prettyprint">
eval `dircolors ~/.dir_colors/dircolors`
</pre>

그 다음 터미네이터의 `Preferences` 에서 `Profiles > Colors`의 `Built-in schemes` 항목을 엽니다. `Solarized dark` 항목이 추가되어 있는 것을 확인할 수 있습니다.

저는 백그라운드 색을 `White on Black`으로 설정하며, `Palette` 항목은 `Solarized`로 설정하고 있습니다.

<br>

## 프롬프트에서 컴퓨터 이름 삭제하는 방법

`.zshrc` 파일 맨 아래에 아래 항목을 추가합니다.

<pre class="prettyprint">
prompt_context() { 
  if [[ "$USER" != "$DEFAULT_USER" || -n "$SSH_CLIENT" ]]; then 
    prompt_segment black default "%(!.%{%F{yellow}%}.)$USER" 
  fi 
}
</pre>

<br>

## .zshrc 예제 

## 예제

<pre class="prettyprint">
# 프롬프트에서 컴퓨터 이름 삭제
prompt_context() { 
  if [[ "$USER" != "$DEFAULT_USER" || -n "$SSH_CLIENT" ]]; then 
    prompt_segment black default "%(!.%{%F{yellow}%}.)$USER" 
  fi 
}

# 터미널 색상 테마 적용
eval `dircolors ~/.dir_colors/dircolors`

# for (i-search)
stty stop undef

# for Java
export JAVA_HOME=/usr/lib/jvm/java-8-oracle

# for Android
export ANDROID_SDK=~/Android/Sdk
export ANDROID_NDK=~/Android/Ndk/android-ndk-r17b
export ANDROID_HOME=$ANDROID_SDK

export PATH=$PATH:$ANDROID_SDK:$ANDROID_TOOLS:$ANDROID_NDK:$ANDROID_HOME/tools:$ANDROID_SDK/platform-tools

# for TERM setting
export TERM=xterm

# for ZSH AutoSuggestion
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE='fg=cyan'
</pre>