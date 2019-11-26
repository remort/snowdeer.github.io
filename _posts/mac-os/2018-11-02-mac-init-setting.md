---
layout: post
title: Mac OS 초기 세팅 (개발용 세팅)

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# Mac OS 초기 세팅 (개발용 세팅)

지극히 개인적인 세팅입니다. 맥 OS 업데이트가 있을 때마다 한 번씩 초기화를 해줬더니 초기화 이후 해줘야 할 일들의 정리가 필요하더군요.

<br>

## 업데이트

모든 OS 공통입니다. 업데이트를 제일 먼저해줍니다. `App Store`를 실행해서 필요한 업데이트 모두 진행합니다.

<br>

## 시스템 설정

### 언어 설정

개발 위주로 사용하다보니 OS 언어를 한글보다는 영문으로 사용하는 것이 더 편리합니다. `Language & Region` 항목으로 가서 선호하는 언어를 `English`로 변경합니다.

<br>

### Dock 설정

Dock 아이콘 크기를 조금 작게 수정하고, `Magnification`은 활성화하고 크기는 `중상` 정도로 세팅합니다.
또한 Dock에서 불필요한 아이콘들은 미리 정리합니다.

<br>

### Keyboard 설정

Keyboard 세팅은 건드릴 부분이 좀 많습니다.

* `Keyboard` 탭에서 `Key Repeat`는 `Fast`로 설정
* `Delay Until Repeat`는 `Short` 쪽에서 1칸 정도 왼쪽으로 설정
* `Touch Bar shows`는 `F1, F2, etc. Keys`로 설정(요즘은 가급적 터치바의 기본 상태를 쓰려고 노력중입니다. 어차피 <kbd>fn</kbd> 키로 펑션키를 실행할 수 있으니깐요.)

`Text` 탭에서 다음 항목을 모두 `Off`로 변경해줍니다.

* Correct spelling automatically
* Capitalize words automatically
* Add period with double-space
* Touch Bar typing suggestions
* Use smart quotes and dashes

`Shortuts`에서 

* `Spotlight` 항목의 `Show Finder search window` 키를 <kbd>Option</kbd> + <kbd>E</kbd>로 수정(최대한 Windows 탐색기와 비슷하게 하기 위해서)
* `Show Spotlight search` 단축키는 <kbd>F12</kbd>로 변경(Mission Control의 Dashboard와 충돌나면, Dashboard 단축키를 사용안함으로 변경)

<br>

### Display 해상도 변경

`System Preferences` 에서 `Displays` 선택합니다. `Resolution` 항목을 취향껏 정해주면 되는데, 저는 최고 해상도로 설정했습니다.

<br>

### Sharing 설정(ssh 서버 on)

다른 PC에서 현재 맥북에 `ssh` 접속을 할 수 있도록 해줍니다.
`Sharing`에서 `Remote Login`은 `On`으로 체크합니다.

<br>

### Touch ID

`Touch ID`는 검지와 중지 2개 정도 등록하면 괜찮을 듯 싶네요.

<br>

## Finder 설정

`Preferences`에서 다음 항목을 수정합니다.

* `General`에서 `New Finder windows show:` 항목을 `Home folder`로 설정
* `Advance`의 `Show all filename extensions` 항목을 `On`으로 설정

<br>

## Homebrew 설치

이제 본격적인 개발용 세팅입니다. 각종 패키지를 편하게 설치하기 위해서는 `Homebrew`를 설치해줍니다.
자세한 것은 [여기](https://brew.sh/)를 참조하면 됩니다.

<pre class="prettyprint">
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
</pre>

설치 후 `brew doctor` 명령어로 정상 동작하는지 확인할 수 있습니다.

<br>

## Command Line Tool 설치

개발용으로는 필수적인 단계입니다. `git` 등 개발용 툴들을 일일이 직접 설치해도 되지만, `xcode`에서 제공하는 `Command Line Tool`을 이용하면
더 편리합니다.

<pre class="prettyprint">
xcode-select --install
</pre>

<br>

## 한글 단축키 변경

기본 한/영 전환은 <kbd>Caps Lock</kbd>입니다. 불편하지 않으면 그냥 써도 되지만 저는 우측 <kbd>Cmd</kbd> 키가 편해서 바꾸기 위해서 키를 바꿉니다.[여기](http://snowdeer.github.io/mac-os/2017/02/14/osx-sierra-right-cmd-language-change/)를 참고해서 설치하면 됩니다.

<br>

## zsh 및 Oh-My-Zsh 설치

리눅스에서도 애용하고 있는 `zsh`와 `Oh-My-zsh`를 설치해줍니다. 

<br>

### zsh 설치

<pre class="prettyprint">
brew install zsh
</pre>

<br>

### Oh-My-Zsh 설치

<pre class="prettyprint">
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
</pre>

초기 세팅은 `nano .zshrc` 실행해서 작성하면 되며, 기존에 Linux에서 사용하던 `.zshrc` 내용을 거의 그대로 사용할 수 있습니다.
제가 주로 사용하는 plugin 2개는 다음 명령어로 설치할 수 있습니다.

<pre class="prettyprint">
git clone https://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
</pre>

`.zshrc` 파일에서 테마는 `agnoster`으로 정합니다.
 
색상 팔레트는 `Preference`에서 원하는 조합으로 지정가능합니다.

폰트 때문에 특수 문자 일부가 깨는 현상 발생할 수 있습니다. `Powerline` 폰트나 `D2Coding` 폰트를 설치하면 해결됩니다.

* Powerline : [다운로드](https://beomi.github.io/others/Ubuntu_Mono_derivative_Powerline.ttf)

<br>

## git 설정

<pre class="prettyprint">
git config --global user.name "snowdeer"
git config --global user.email "snowdeer0314@gmail.com"
git config --global core.precomposeunicode true
git config --global core.quotepath false
</pre>

<br>

## JDK (Java Runtime) 설치

[여기](https://www.oracle.com/technetwork/java/javase/downloads/index.html?ssSourceSiteId=otnjp)에서 `dmg` 파일을 다운로드해서
설치합니다.

<br>

## 필요한 프로그램 설치

* AppCleaner: [http://freemacsoft.net/appcleaner/](http://freemacsoft.net/appcleaner/)
* Dropbox: [https://www.dropbox.com/ko/downloadin](https://www.dropbox.com/ko/downloading)
* 1Password: [https://1password.com/ko/downloads/mac/](https://1password.com/ko/downloads/mac/)
* iTerm2: [https://iterm2.com](https://iterm2.com)
* Visual Studio Code: [https://code.visualstudio.com](https://code.visualstudio.com)
* Android Studio: [https://developer.android.com/studio/](https://developer.android.com/studio/)
* Android NDK: [https://developer.android.com/ndk/downloads/?hl=ko](https://developer.android.com/ndk/downloads/?hl=ko)
* Android File Transfer: [https://www.android.com/filetransfer/](https://www.android.com/filetransfer/)
* PyCharm: [https://www.jetbrains.com/pycharm/download/](https://www.jetbrains.com/pycharm/download/)
* CLion: [https://www.jetbrains.com/clion/download/#section=mac](https://www.jetbrains.com/clion/download/#section=mac)
* IntelliJ: [https://www.jetbrains.com/idea/download/#section=mac](https://www.jetbrains.com/idea/download/#section=mac)
* Wireshark: [https://www.wireshark.org/#download](https://www.wireshark.org/#download)

* Hidden Bar: App Store - 설치후 <kbd>Command</kbd> 키를 이용해서 상단바의 아이콘을 왼쪽 영역으로 이동

<br>

## ip 설치

Mac에는 `ip` 명령어가 없기 때문에 `ip addr` 같은 명령어를 사용할 수 없습니다. 다음 명령어로 설치해줍니다.

<pre class="prettyprint">
brew install iproute2mac
</pre>