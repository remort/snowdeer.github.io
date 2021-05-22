---
layout: post
title: SW 품질속성 예제
category: S/W Architecture
permalink: /sw-architecture/:year/:month/:day/:title/

tag: [설계, 품질속성]
---

# 품질속성 6가지 항목

![image](/assets/2017-02-15-sw-quality-attribute/01.jpg)

각각의 항목에 대한 구체적인 예시들은 다음과 같습니다.

<br>

## 가용성 시나리오 예시

항목 | 값
--- | ---
Source | 생명신호 모니터링
Stimulus | 서버가 반응하지 않음
Artifact | 프로세스
Environment | 정상적인 운영
Response | 운영자에게 보고하고 운영을 계속한다.
Measure | 중단 시간 없음

<br>

## 상호운영성 예시

항목 | 값
--- | ---
Source | 운송 정보 시스템
Stimulus | 현재 자극 위치 전송
Artifact | 교통 통제 시스템
Environment | 런타임 이전에 알려진 시스템
Response | 트래픽 모니터 시스템은 현재 위치와 다른 정보를 결합한다.<br>구글 맵스 위에 올려놓고 브로드캐스트한다.
Measure | 99.9% 정확하게 시간이 포함된 우리 정보

<br>

## 변경 용이성 시나리오

항목 | 값
--- | ---
Source | 개발자
Stimulus | UI를 변경하기 원한다.
Artifact | 코드
Environment | 설계 시
Response | 변경을 하고 단위 테스트를 수행한다.
Measure | 3시간 이내

<br>

## 성능 시나리오

항목 | 값
--- | ---
Source | 사용자
Stimulus | 트랜잭션 시작
Artifact | 시스템
Environment | 정상 운영
Response | 트랜잭션이 처리됨
Measure | 평균 2초의 지연 시간

<br>

## 보안 시나리오

항목 | 값
--- | ---
Source | 원격 위치에 있는 불만을 가진 직원
Stimulus | 임금률 변경 시도
Artifact | 시스템 안에 있는 데이터
Environment | 정상 운영
Response | 시스템은 감사 트레일을 유지한다.
Measure | 하루 안에 정확한 데이터로 복구되고, 공격의 근원을 식별한다.

<br>

## 테스트 용이성 시나리오

항목 | 값
--- | ---
Source | 단위 테스터
Stimulus | 코드 단위 완료
Artifact | 코드 단위
Environment | 개발
Response | 수집된 결과
Measure | 3시간 안에 85% 경로 커버리지

<br>

## 사용 편의성 시나리오

항목 | 값
--- | ---
Source | 사용자
Stimulus | 새로운 App을 다운로드한다.
Artifact | 시스템
Environment | 런타임
Response | 사용자가 생산적으로 App을 사용한다.
Measure | 측정 2분 동안 시험한다.

