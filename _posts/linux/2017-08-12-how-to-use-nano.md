---
layout: post
title: Nano 사용법
category: Linux
tag: [Linux 명령어]
---
# Nano

리눅스 쉘 기반에서 사용할 수 있는 텍스트 편집기 프로그램 중 하나입니다. 개인적으로는 `vi` 보다 사용하기 편리한 것 같아서 애용하고 있습니다.

<br>

## nano 실행 방법

명령어 | 설명
--- | ---
nano memo.txt | memo.txt 파일을 Open
nano -B memo.txt | 저장 전에 이전 파일을 ~.filename 으로 백업함
nano -m memo.txt | 마우스를 이용해서 Cursor 이동 지원
nano -55 memo.txt | memo.txt 파일의 55번 라인부터 시작

<br>

## 편집 모드에서 단축키

편집 모드에서는 방향키나 <kbd>Ins</kbd>, <kbd>Del</kbd>, <kbd>Enter</kbd> 등 다양한 키들을 사용할 수 있습니다.

단축키 | 설명
--- | ---
<kbd>Ctrl</kbd> + <kbd>O</kbd> 또는 <kbd>F3</kbd> | 저장
<kbd>Ctrl</kbd> + <kbd>X</kbd> 또는 <kbd>F2</kbd> | nano 종료 및 저장
<kbd>Ctrl</kbd> + <kbd>W</kbd> 또는 <kbd>F6</kbd> | 검색
<kbd>Ctrl</kbd> + <kbd>\</kbd> | 검색 및 Replace
<kbd>Ctrl</kbd> + <kbd>G</kbd> | 도움말 실행

<br>

## 편집에 관련된 단축키

단축키 | 설명
--- | ---
<kbd>Ctrl</kbd> + <kbd>K</kbd> 또는 <kbd>F9</kbd> | 현재 Line 또는 선택한 텍스트 잘라내기
<kbd>Ctrl</kbd> + <kbd>U</kbd> 또는 <kbd>F10</kbd> | 붙여넣기
<kbd>Ctrl</kbd> + <kbd>Y</kbd> 또는 <kbd>Page Up</kbd>| 이전 화면
<kbd>Ctrl</kbd> + <kbd>V</kbd> 또는 <kbd>Page Down</kbd>| 다음 화면
<kbd>Ctrl</kbd> + <kbd>-</kbd> | 원하는 Line으로 이동
<kbd>Alt</kbd> + <kbd>(</kbd> | 현재 문단의 시작으로
<kbd>Alt</kbd> + <kbd>)</kbd> | 현재 문단의 끝으로
<kbd>Alt</kbd> + <kbd>\</kbd> | 파일의 처음으로
<kbd>Alt</kbd> + <kbd>/</kbd> | 파일의 끝으로