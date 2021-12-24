---
layout: post
title: M1 MacBook 초기 세팅 (개발용 세팅)

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# M1 MacBook 초기 세팅 (개발용 세팅)

M1 맥북의 초기 설정입니다. 아무래도 기존 Intel Mac OS 때와 차이가 있을 수 있어서 하나씩 다시 점검하면서 설정을 했습니다.
M1 Pro 14인치 기준입니다.

<br>

## 시스템 설정

### 언어 설정
개발용으로 사용하기 때문에 영어가 더 편리할 수 있습니다. `Language & Region`에서 선호 언어를 영어로 설정합니다.
아래부터는 영문 버전 메뉴 이름을 사용하겠습니다.

### Dock & Menu Bar 설정
- Dock 아이콘 크기는 조금 작게 설정
- `Magnification`을 체크하고 크기는 중상 정도로 설정
- `Show recent applications in Dock` 체크 해제
- `Menu Bar`의 `Automatically hide and show the menubar ...`는 둘 다 On

### Keyboard 설정

#### Keyboard 탭
- Key Repeat는 `Fast`로 설정
- Delay Until Repeat는 `Short`에서 1칸 왼쪽으로 설정
- Press fn(지구본) to `Do Nothing`으로 설정

#### Text 탭
- `Correct spelling automatically` 항목 Off
- `Capitalize words automatically` 항목 Off
- `Add period with double-space` 항목 Off
- `Use smart quotes and dashes` 항목 Off

#### Shortcuts 탭
- `Spotlight`의 `Show Finder search window`를 <kbd>Option</kbd> + <kbd>E</kbd>로 변경(윈도우와 비슷하게 하기 위해)
- `Show Spotlight search`는 <kbd>F12</kbd>로 변경

### Display 설정
14인치 맥북 기준으로 아무 설정도 건드리지 않았습니다.
예전 15인치 맥북에서는 `Resolution`을 최고로 조정했었는데, 이번 14인치 맥북에서는 최고 해상도가 글씨가 조금 작은 듯 하여
기본으로 설정했습니다.

### Touch ID 설정
터치 ID는 3개까지 등록 가능한데, 저는 오른 손 검지로 2번, 중지로 1번 등록했습니다.

<br>

## Finder 설정
`Preference` 메뉴로 가서 
- General 탭에서 `New Finder windows show:` 항목을 `Home folder`로 설정(저는 Home이 `snowdeer`로 되어 있네요.)
- Advance 탭의 `Show all filename extensions` 항목을 On
- Sidebar 탭에서 필요한 항목들만 보이도록 설정
- 실제 Finder를 실행해서 Sidebar의 순서를 변경

<br>

## D2Coding 폰트 설치
향후 `Oh-my-zsh` 설정에서 아마 폰트가 깨질겁니다. `Powerline` 폰트나 `D2Coding` 폰트를 설치하면 해결됩니다.
그래서 미리 `D2Coding` 폰트를 설치해놓도록 하겠습니다.

다운로드: [https://github.com/naver/d2codingfont](https://github.com/naver/d2codingfont)

<br>

## iTerm2 설치
주로 작업을 터미널에서 하기 때문에 `iTerm2`를 설치하도록 합니다.
다운로드: [https://iterm2.com](https://iterm2.com)

그리고 `Preferences`를 실행해서 다음 항목을 설정합니다.

- `Profiles` > `Text` > `Font`를 기본 `Monaco`에서 `D2 Coding`으로 변경
- `Profiles` > `Colors` > `Color Presets`를 `Pastel (Dark Backgrond)` 또는 `Tango Dark`로 변경
- `Window` > `Columns`를 `80`에서 `120`로 `Rows`를 `30`으로 변경(14인치 맥북 기준입니다.)

<br>

## Visual Studio Code 설치
다운로드: [https://code.visualstudio.com/#](https://code.visualstudio.com/#)

그리고 실행한다음 <kbd>Command</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd>를 입력해서 Command Palette를 실행한 다음
`shell`이라고 입력하면, `Shell Command: Install 'code' in PATH`를 선택합니다. 그러면 이제 터미널에서 `code` 명령어를 이용해서
vscode를 실행할 수 있습니다.

<br>

## 한글 단축키 변경

기본 한/영 전환은 <kbd>Caps Lock</kbd>입니다. 불편하지 않으면 그냥 써도 되지만 저는 우측 <kbd>Cmd</kbd> 키가 편해서 바꾸기 위해서 키를 바꿉니다.[여기](http://snowdeer.github.io/mac-os/2021/12/22/macos-m1-remapping-command-key-for-change-korean-english-key/)를 참고해서 설치하면 됩니다.

<br>

## Homebrew 설치

<pre class="prettyprint">
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
</pre>

설치 후 아래 명령어도 실행합니다.

<pre class="prettyprint">
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/snowdeer/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
</pre>

<br>

## zsh 및 Oh-My-Zsh 설치

### zsh 설치
이제 맥 기본 쉘이 `zsh`이므로 바로 `Oh-My-Zsh` 설치로 넘어가면 됩니다.

### Oh-My-Zsh 설치
<pre class="prettyprint">
sh -c "$(curl -fsSL https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh)"
</pre>

### zsh 플러그인 설치

<pre class="prettyprint">
git clone https://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
</pre>

### ~/.zshrc 파일 편집
`~/.zshrc` 파일을 열어서 다음 부분을 찾아서 변경해줍니다.

<pre class="prettyprint">
ZSH_THEME="agnoster"
plugins=(
  git
  zsh-autosuggestions
  zsh-syntax-highlighting
)
</pre>

그리고 파일 끝 부분에 아래 내용도 추가합니다.

{%raw%}
<pre class="prettyprint">
# for (i-search)
stty stop undef

# 프롬프트에서 컴퓨터 이름 삭제
prompt_context() { 
  if [[ "$USER" != "$DEFAULT_USER" || -n "$SSH_CLIENT" ]]; then 
    prompt_segment black default "%(!.%{%F{yellow}%}.)$USER" 
  fi 
}
</pre>
{%endraw%}

<br>

## git 설정

<pre class="prettyprint">
git config --global user.name "snowdeer"
git config --global user.email "snowdeer0314@gmail.com"
git config --global core.precomposeunicode true
git config --global core.quotepath false
</pre>

<br>

## 필요한 프로그램 설치
- AppCleaner: [http://freemacsoft.net/appcleaner/](http://freemacsoft.net/appcleaner/)
- iTerm2: [https://iterm2.com](https://iterm2.com)
- Visual Studio Code: [https://code.visualstudio.com](https://code.visualstudio.com)
- PyCharm: [https://www.jetbrains.com/pycharm/download/](https://www.jetbrains.com/pycharm/download/)
- IntelliJ: [https://www.jetbrains.com/idea/download/#section=mac](https://www.jetbrains.com/idea/download/#section=mac)
- WebStorm: [https://www.jetbrains.com/webstorm/download/#section=mac](https://www.jetbrains.com/webstorm/download/#section=mac)
- Clion: [https://www.jetbrains.com/clion/download/#section=mac](https://www.jetbrains.com/clion/download/#section=mac)

아래는 사파리 툴체인이나 크롬을 이용해서 패스워드를 관리하지만, 기존에 사놓은 제품이 아까워서 설치해줍니다. ㅜㅜ;
- Dropbox: [https://www.dropbox.com/downloading?src=index](https://www.dropbox.com/downloading?src=index)
- 1Password: [https://1password.com/ko/downloads/mac/](https://1password.com/ko/downloads/mac/) 에서 기존 버전 6.8.9 다운로드

아래는 슬프게도 M1 칩을 지원하지 않아서 설치 못하는 어플이네요. ㅜㅜ; 
TotalFinder는 가장 유용하게 사용한 어플인데 아쉽습니다.
- TotalFinder: https://totalfinder.binaryage.com

그 외 `brew`를 이용해서 다음 프로그램들도 설치합니다.

<pre class="prettyprint">
brew install iproute2mac
</pre>

App Store에서도 필요한 프로그램들을 설치합니다.

- Unicorn Blocker:Adblock
- Lumafusion

