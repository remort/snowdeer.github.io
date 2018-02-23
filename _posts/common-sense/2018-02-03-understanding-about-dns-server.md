---
layout: post
title: DNS 서버
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어, 네트워크]
---
# DNS 서버의 종류

DNS(Domain Name System) 서비스를 제공하는 서버를 DNS 서버라고 합니다. DNS 서버는 '캐시 서버(Cache Server)'와 '컨텐츠 서버(Contents Server)'로 나누어집니다.

캐시 서버는 로컬 네트워크(LAN)안에 있는 클라이언트로부터 요청을 받아 클라이언트를 대신해서 인터넷에 요청을 하는 DNS 서버로 클라리언트에서 인터넷에 접근할 때 사용합니다.

컨텐츠 서버는 외부 호스트로부터 자신이 관리하는 도메인에 관한 요청 조회를 받는 DNS 서버입니다. 자신의 도메인내의 호스트명은 'zone 파일'이라는 데이터베이스로 관리하고 있습니다.