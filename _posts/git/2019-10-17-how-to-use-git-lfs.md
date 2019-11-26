---
layout: post
title: Git LFS(Large File Storage) 사용법
category: Git
tag: [git]
---
# LFS(Large File Storage)

일반적으로 Git은 파일 한 개 용량이 100MB 까지만 업로드 가능합니다. 그 이상의 파일을 올리고 싶을 때는
LFS(Large File Storage)를 이용해야 합니다. 

LFS 설치 방법은 다음과 같습니다.

<br>

## LFS 설치 방법

Ubuntun에서는 다음 명령어를 이용해서 설치합니다. [여기](https://packagecloud.io/github/git-lfs/install)를 참고하세요.

<pre class="prettyprint">
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

sudo apt install git-lfs
</pre>

<br>

MAC에서는 `brew`를 이용해서 설치합니다. 또는 [여기](https://git-lfs.github.com)에서 다운로드해서 설치하세요.

<pre class="prettyprint">
brew install git-lfs
</pre>

<br>

## LFS 사용 방법

LFS를 사용하려는 프로젝트 루트에 가서 다음 명령어를 입력합니다. 여기서는 안드로이드 빌드 결과물인 `apk` 확장자로 끝나는 파일에 대해
적용을 해봅니다. 

<pre class="prettyprint">
git lfs install
git lfs track "*.apk"
git add .gitattributes
</pre>

그 이후 평소와 같이 파일을 `commit` 하고 `push` 하면 됩니다. 

터미널에서 `git lfs` 명령어를 내려도 사용법을 간편하게 조회할 수 있습니다.