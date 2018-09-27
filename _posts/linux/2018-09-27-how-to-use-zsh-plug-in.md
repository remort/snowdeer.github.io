---
layout: post
title: ZSH Plug-in 사용 방법
category: Linux
tag: [리눅스]
---
# ZSH Plug-in 사용 방법

`ZSH`에는 수 많은 플러그인(Plug-in) 들이 있습니다. 그런데, 그 중에서 쓸만한 것들이 아주 많은 것 같지는 않습니다. 다음은 개인적으로 쓰고 있는, 범용적으로 쓰일 수 있는 플러그인들입니다.

`ZSH` 플러그인 설치는 `~/.oh-my-zsh/custom/plugins` 디렉토리에 하면 되며, 사용 유무는 `~/.zshrc` 파일에서 설정 할 수 있습니다.

<br>

## 플러그인 설치 예시

<pre class="prettyprint">
cd ${ZSH_CUSTOM1:-$ZSH/custom}/plugins

git clone https://github.com/djui/alias-tips.git
git clone https://github.com/zsh-users/zsh-autosuggestions $ZSH_CUSTOM/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
</pre>

그 이후 `~/.zshrc` 파일에 다음과 같이 설정합니다.

<pre class="prettyprint">
plugins=(
  git
  alias-tips
  zsh-autosuggestions
  zsh-syntax-highlighting
)
</pre>

그 외에도 저는 [`wd`](https://github.com/mfaerevaag/wd) 같은 플러그인도 사용하고 있습니다.

