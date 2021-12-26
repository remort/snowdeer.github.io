---
layout: post
title: M1에 brew 실행 오류 발생 해결법

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# M1에 brew 실행 오류 발생 해결법

M1 맥북에서 `brew` 명령어를 실행했을 때 오류가 발생한 경우에는 `brew doctor`로 문제를 해결해줍니다.
저같은 경우는 `brew install tree`에서 오류가 발생했네요.

<pre class="prettyprint">
$ brew install tree

Warning: No available formula with the name "tree".
==> Searching for similarly named formulae...
Error: No similarly named formulae found.
==> Searching for a previously deleted formula (in the last month)...
Error: No previously deleted formula found.
==> Searching taps on GitHub...
Error: No formulae found in taps.
</pre>

이 경우 `brew doctor`를 실행하면 다음과 같이 현재 문제점을 알려줍니다.

<pre class="prettyprint">
$ brew doctor

Please note that these warnings are just used to help the Homebrew maintainers
with debugging if you file an issue. If everything you use Homebrew for is
working fine: please don't worry or file an issue; just ignore this. Thanks!

Warning: A newer Command Line Tools release is available.
Update them from Software Update in System Preferences or run:
  softwareupdate --all --install --force

If that doesn't show you any updates, run:
  sudo rm -rf /Library/Developer/CommandLineTools
  sudo xcode-select --install

Alternatively, manually download them from:
  https://developer.apple.com/download/all/.
You should download the Command Line Tools for Xcode 13.1.


Warning: Homebrew/homebrew-core was not tapped properly! Run:
  rm -rf "/opt/homebrew/Library/Taps/homebrew/homebrew-core"
  brew tap homebrew/core

Warning: Some taps are not on the default git origin branch and may not receive
updates. If this is a surprise to you, check out the default branch with:
  git -C $(brew --repo homebrew/core) checkout master
</pre>

저 같은 경우 `Homebrew/homebrew-core was not tapped` 문제가 있어 다음 명령어를 실행했습니다.

<pre class="prettyprint">
$ rm -rf "/opt/homebrew/Library/Taps/homebrew/homebrew-core"
$ brew tap homebrew/core
</pre>