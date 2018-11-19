---
layout: post
title: 유용한 탐색 프로그램 fzf 사용하기

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# fzf 사용하기

`Fuzzy Finder`의 약어로, 터미널상에서 `find` 명령어를 대신할 수 있는 명령어입니다. 사용 방법이 쉬우며 아주 강력해서 많이 애용하고 있습니다.

<br>

## 설치 방법

<pre class="prettyprint">
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install
</pre>

설치한 다음 터미널에서 `fzf`를 입력해보면 어떤 프로그램인지 바로 알 수 있을 것입니다.

<br>

## 타 프로그램과 연동

`fzf`의 강력한 점은 단독 실행보다 타 프로그램과의 연동에서 발휘됩니다. 
다음과 같이 `fzf`의 결과값을 받아서 다른 프로그램과 연동해서 사용할 수 있습니다.

<pre class="prettyprint">
vim $(fzf)
nano $(fzf)

vim $(fzf --height 60%)
vim $(fzf --height 30% --reverse)

# Default 옵션 설정도 가능합니다.
export FZF_DEFAULT_OPTS='--height 40% --layout=reverse --border'
</pre>

`fzf`의 Finder 내에서는 커서나 마우스로 파일을 선택하거나 스크롤을 할 수 있습니다. 
여러 항목을 선택하고 싶은 경우에는 `-m` 옵션을 이용해서 실행하면 되며, <kbd>Tab</kbd> 및 <kbd>Shift</kbd> + <kbd>Tab</kbd> 키를 이용해서
여러 파일을 선택할 수 있습니다.

<br>

### 자동완성

`fzf`의 강점 중 자동완성 기능이 있는데, 아주 강력합니다. 사용법은 `[명령어] [경로][패턴]**[TAB 키]`이며, 사용 예제는 다음과 같습니다.

~~~
nano **<TAB>
vim **<TAB>
nano text**<TAB>
ls ./s**<TAB>
~~~

그 외에도 

~~~
kill -9 <TAB>

ssh <TAB>
~~~

등과 같이 활용할 수도 있습니다.

<br>

### preview 기능

`fzf`는 파일 내용을 미리 볼 수 있는 `preview` 기능을 제공합니다.
다음 함수를 `.zshrc`에 추가합시다. 터미널에서 `fzfv`를 입력하면 실행할 수 있습니다.

<pre class="prettyprint">
function fzfv()
{
    fzf --preview '[[ $(file --mime {}) =~ binary ]] &&
                 echo {} is a binary file ||
                 (cat {}) 2> /dev/null | head -500'
}
</pre>