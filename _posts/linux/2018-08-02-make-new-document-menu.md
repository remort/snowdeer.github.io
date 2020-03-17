---
layout: post
title: 마우스 오른 버튼 메뉴에 New Document 항목 생성하기
category: Linux
tag: [리눅스, Ubuntu]
---
# New Document 옵션 살리기 on Ubuntu 18.04

Ubuntu 16.04 까지는 마우스 오른 버튼 메뉴에 `New Document` 항목이 있었는데 Ubuntu 18.04에서는 이 항목이 사라졌습니다. 다음 방법을 이용해서 `New Document` 항목을 살릴 수 있습니다.

터미널을 열고 다음 명령어를 입력한다.

<pre class="prettyprint">
touch ~/Templates/"Empty Document.txt"
</pre>

디렉토리 명에서 알 수 있듯이, 위에서 생성한 문서 안에 특정 문구들을 저장해놓으면 새로운 문서를 생성할 때마다 해당 내용들을 템플릿처럼 사용할 수 있습니다.