---
layout: post
title: ZSH Plug-in 소개
category: Linux
tag: [리눅스, zsh]
---
# ZSH Plug-in 소개

## history-substring-search

`zsh` 셀에서 키워드를 입력한 다음 화살표 위 방향 키로 입력된 키워드가 포함된 과거 실행 명령어를 보여주는 유용한 플러그인입니다.

<pre class="prettyprint">
 git clone https://github.com/zsh-users/zsh-history-substring-search ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-history-substring-search
</pre>

## syntax-highlighting

<pre class="prettyprint">
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
</pre>

## ~/.zshrc 설정

<pre class="prettyprint">
plugins=(
  git
  alias-tips
  zsh-autosuggestions
  zsh-syntax-highlighting
)
</pre>
