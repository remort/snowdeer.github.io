---
layout: post
title: Git pre-commit 사용법
category: Git
tag: [git]
---
# pre-commit

이름에서 볼 수 있듯이 `Git commit`을 수행하기 전에 자동으로 특정 작업을 수행하도록 해주는 기능입니다. 
보통 `formatter` 또는 `linter` 등을 실행해서 코드의 잠재적 문제 발견 또는 일관성있는 포맷을
유지하게 해줍니다.

<br>

## 설치 방법

`pre-commit`는 파이썬 패키지를 이용해서 설치할 수 있습니다.

<pre class="prettyprint">
$ pip3 install pre-commit

Collecting pre-commit
  Downloading pre_commit-2.12.1-py2.py3-none-any.whl (189 kB)
     |████████████████████████████████| 189 kB 813 kB/s
Collecting virtualenv>=20.0.8
  Downloading virtualenv-20.4.4-py2.py3-none-any.whl (7.2 MB)
     |████████████████████████████████| 7.2 MB 869 kB/s
Requirement already satisfied: toml in /usr/local/lib/python3.8/site-packages (from pre-commit) (0.10.2)
Collecting cfgv>=2.0.0
  Downloading cfgv-3.2.0-py2.py3-none-any.whl (7.3 kB)
Collecting nodeenv>=0.11.1
  Downloading nodeenv-1.6.0-py2.py3-none-any.whl (21 kB)
Requirement already satisfied: pyyaml>=5.1 in /usr/local/lib/python3.8/site-packages (from pre-commit) (5.4.1)
Collecting identify>=1.0.0
  Downloading identify-2.2.4-py2.py3-none-any.whl (98 kB)
     |████████████████████████████████| 98 kB 17.3 MB/s
Collecting filelock<4,>=3.0.0
  Downloading filelock-3.0.12-py3-none-any.whl (7.6 kB)
Collecting appdirs<2,>=1.4.3
  Downloading appdirs-1.4.4-py2.py3-none-any.whl (9.6 kB)
Requirement already satisfied: distlib<1,>=0.3.1 in /usr/local/lib/python3.8/site-packages (from virtualenv>=20.0.8->pre-commit) (0.3.1)
Requirement already satisfied: six<2,>=1.9.0 in /usr/local/lib/python3.8/site-packages (from virtualenv>=20.0.8->pre-commit) (1.15.0)
Installing collected packages: filelock, appdirs, virtualenv, nodeenv, identify, cfgv, pre-commit
Successfully installed appdirs-1.4.4 cfgv-3.2.0 filelock-3.0.12 identify-2.2.4 nodeenv-1.6.0 pre-commit-2.12.1 virtualenv-20.4.4
</pre>

MacOS에서는 `brew`를 이용해서도 설치 가능하지만, 그냥 `pip3`를 이용하는게 더 편한 듯 싶습니다.

<br>

## 버전 확인

<pre class="prettyprint">
$ pre-commit -V

pre-commit 2.12.1
</pre>

<br>

## 설정

`pre-commit`는 `.pre-commit-config.yaml` 설정 파일을 필요로 합니다.
아래 명령어를 이용해서 `sample-config`라는 템플릿으로 설정 파일을 만들 수 있습니다.
`git add` 명령어로 프로젝트에 추가할 예정이니, 아래 명령어는 `pre-commit`를 적용하려는 프로젝트의 디렉토리에서 실행하세요.

<pre class="prettyprint">
$ pre-commit sample-config > .pre-commit-config.yaml
</pre>

<br>

### .pre-commit-config.yaml

해당 설정 파일은 다음과 같은 내용이 작성되어 있습니다.

<pre class="prettyprint">
# See https://pre-commit.com for more information
# See https://pre-commit.com/hooks.html for more hooks
repos:
-   repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v3.2.0
    hooks:
    -   id: trailing-whitespace
    -   id: end-of-file-fixer
    -   id: check-yaml
    -   id: check-added-large-files
</pre>

총 4개의 `hook`이 설정되어 있는 것을 볼 수 있습니다.

<br>

## 실행

이제 `pre-commit run` 명령을 실행해봅니다. 처음에는 해당 repo로부터 다운로드하는 시간이 있어서 
약간의 시간이 걸립니다.

<pre class="prettyprint">
$ pre-commit run

[INFO] Initializing environment for https://github.com/pre-commit/pre-commit-hooks.
[INFO] Installing environment for https://github.com/pre-commit/pre-commit-hooks.
[INFO] Once installed this environment will be reused.
[INFO] This may take a few minutes...
Trim Trailing Whitespace.............................(no files to check)Skipped
Fix End of Files.....................................(no files to check)Skipped
Check Yaml...........................................(no files to check)Skipped
Check for added large files..........................(no files to check)Skipped
</pre>

그 이후 `pre-commit run -a` 명령어를 이용해서 모든 파일들을 한 번 검사해봅니다.

<pre class="prettyprint">
$ pre-commit run -a

...
</pre>

다음 명령어를 이용해서 `.pre-commit-config.yaml` 파일을 git 에 추가해줍니다.

<pre class="prettyprint">
$ git add .pre-commit-config.yaml

$ git commit -m "pre-commit 적용"
</pre>

<br>

## git hook에 등록

마지막으로 `git commit` 할 때 자동으로 `pre-commit`가 실행되도록 합니다. 
아래 명령어를 이용해서 `git hook`에 등록할 수 있습니다.

<pre class="prettyprint">
$ pre-commit install
</pre>
