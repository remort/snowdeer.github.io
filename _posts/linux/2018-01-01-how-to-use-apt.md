---
layout: post
title: apt 명령어 사용법
category: Linux
tag: [리눅스 명령어, Ubuntu]
---
# apt 명령어 사용법

명령어 | 설명
---|---
apt-cache search [keyword] | 키워드를 포함하고 있는 패키지 명과 설명 출력. 대소문자 구분 없음
apt-cache show [package name] | 해당 패키지의 정보 출력
apt-get update | `/etc/apt/sources.list`를 참조해서 패키지 데이터베이스를 업데이트. 만약 `source.list`를 변경했다면 반드시 실행해줘야 하는 명령어
apt-get upgrade | 설치되어 있는 모든 패키지를 최신 버전으로 업그레이드
apt-get dist-upgrade | 전체 시스템을 새로운 버전으로 업그레이드. 여기에는 패키지 삭제도 포함됨. 시스템 업그레이드 용도로는 잘 권장하지 않는 방법
apt-get install [package name] | 해당 패키지를 다운받아서 설치
apt-get -d install [package name] | 해당 패키지를 `/var/cache/apt/archives` 디렉토리에 다운만 받고 설치하지 않음
apt-get autoclean | 불완전하게 다운로드된 패키지나 오래된 패키지를 삭제함
apt-get clean | `/var/cache/apt/archives`에 저장된 캐시 파일들을 모두 삭제
apt-get remove [package name] | 해당 패키지와 그 설정 파일들을 보두 삭제. `--purge` 옵션은 설정 파일을 보존하는 옵션