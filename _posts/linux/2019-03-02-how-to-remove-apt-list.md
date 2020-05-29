---
layout: post
title: apt 목록 제거하기

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# apt 목록 제거

여러 프로그램들을 설치하다보면 `apt update` 목록에 원하지도 않는 목록들이 추가되서 너덜너덜해지는 경우가 생깁니다. 그 중에 어떤 목록은 더 이상 유효하지 않아서 지우고 싶은 경우도 생기구요.

`/etc/apt` 위치에 가면 해당 서버들의 리스트를 가져올 수 있습니다.