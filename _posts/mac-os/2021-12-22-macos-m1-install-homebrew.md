---
layout: post
title: M1 Homebrew 설치

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# M1 Homebrew 설치

[Homebrew 3.0](https://brew.sh/2021/02/05/homebrew-3.0.0/)부터 M1(실리콘) MacOS를 지원합니다.
기존에 Homebrew를 설치하는 방법과 동일합니다. 다음 명령어를 이용해서 설치할 수 있습니다.

<pre class="prettyprint">
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
</pre>

설치 후 다음과 같은 메시지가 출력됩니다.

<pre class="prettyprint">
...

Press RETURN to continue or any other key to abort:
==> /usr/bin/sudo /usr/sbin/chown -R snowdeer:admin /opt/homebrew
==> Downloading and installing Homebrew...
HEAD is now at e970bb187 Merge pull request #12616 from SMillerDev/chore/docs/common_cask_issues
error: Not a valid ref: refs/remotes/origin/master
fatal: ambiguous argument 'refs/remotes/origin/master': unknown revision or path not in the working tree.
Use '--' to separate paths from revisions, like this:
'git &lt;command&gt; [&lt;revision&gt;...] -- [&lt;file&gt;...]'
fatal: Could not resolve HEAD to a revision
Warning: /opt/homebrew/bin is not in your PATH.
  Instructions on how to configure your shell for Homebrew
  can be found in the 'Next steps' section below.
==> Installation successful!

==> Homebrew has enabled anonymous aggregate formulae and cask analytics.
Read the analytics documentation (and how to opt-out) here:
  https://docs.brew.sh/Analytics
No analytics data has been sent yet (nor will any be during this install run).

==> Homebrew is run entirely by unpaid volunteers. Please consider donating:
  https://github.com/Homebrew/brew#donations

==> Next steps:
- Run these two commands in your terminal to add Homebrew to your PATH:
    echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/snowdeer/.zprofile
    eval "$(/opt/homebrew/bin/brew shellenv)"
- Run brew help to get started
- Further documentation:
    https://docs.brew.sh
</pre>

설치 후 아래의 `Next steps`에 있는 두 명령어를 실행해야 `brew` 명령어를 실행할 수 있습니다.

<pre class="prettyprint">
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> /Users/snowdeer/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
</pre>
