---
layout: post
title: 우분투(Ubuntu) 20.04 한글 키보드 설치
category: Linux
tag: [리눅스 설정, Ubuntu]
---
# iBus 기반 한글 키보드 설치

Ubuntu 20.04 LTS 버전 기준으로 한글 키보드를 설치하는 방법입니다.
Ubuntu 18.04 이후부터 기본 한글어 입력기가 다시 `ibus`로 돌아왔습니다. 
그래서 저는 그냥 기본 입력기를 사용하고 있습니다. 

`ibus`를 이용한 한글 입력기 활성화 방법은 다음과 같습니다.

* 메뉴에서 `Language Support` 실행 → 필요한 파일들 자동으로 설치
* 메뉴에서 `Region & Language` 실행
* `Input Sources` 항목에서 기본으로 잡혀있던 `English`는 삭제하고, `Korean(Hangul)` 선택
* 만약, `Korean(Hangul)`이 뜨지 않고 `Korean`만 표시되는 경우 재부팅을 하면 됩니다.
* `Korean(Hangul)` 항목 오른쪽의 설정 버튼 클릭
* `Hangul Toggle Key`의 `Add` 버튼을 누르고 <kbd>한글</kbd> 키 입력(ALT_R 또는 Hangul로 표시됩니다.)