---
layout: post
title: 요구 공학 Keywords
category: 소프트웨어 설계
tag: [requirement engineering]
---

요구 공학의 키워드들만 간단히 정리해보았습니다.

<br> 

## 요구 공학

요구 사항 추출, 분석, 기술, 검증, 유지 보수 및 관리를 포함한 제본 공정에
대한 체계적 접근 분야

* 기능 요구사항(Functional Requirements)
* 비기능 요구사항(Non-functional Requirements) : Performance, Reliability, Security, Portability 등
기능 요구사항 보다 더 중대한 문제가 될 수 있음
* 도메인 요구사항(Domain Requirements)

<br>

## 요구 공학 프로세스 

Process Activities
* 요구사항 발견
* 요구사항 분류
* 우선 순위 및 협상
* 문서화

MBASE
: Model Based Architecture Software Engineering

Model
: Includes Product models, Process models, Property models, Success models.

Model Clash
: Model간 충돌

Model Integration
: Choosing and/or reengineering models to reconcile their underlying assumptions.

LCO 
: Life Cycle Object

LCA
: Life Cycle Architecture

<br>

## 요구 사항 추출(Requirements Elicitation)

* 자신이 스마트한 것 처럼 보여주려고 노력하지 마라
* 자신이 고객(Stakeholder)을 스마트하다고 생각하는 것을 보여줘라.

요구사항 추출 기법
* Interview
* Brain Storming
* Observation
* Presentation-Based Approaches

ViRE
: Value Innovative Requirements Engineering(가치 기반 요구 공학)

ERRC analysis
: (Value를) Eliminate, Reduce, Raise, Create

<br>

## 요구 사항 협상

WinWin Approach

Use 4-step solution approach
* Seperate the people from the problem
* Focus on interests, not positions (Ask Why? Why not?)
* Invent options for mutual gain
* Insist on using objective criteria

Key Concept : Win Condition, Issue, Option, Aggreement

![Image]({{ site.baseurl }}/assets/2016-08-19-requirement-engineering-keywords/winwin.png)

<br> 

## 요구 사항 명세

* 정확성(Correct)
* 완전성(Complete) : 모든 이해 관계자의 요구를 충족함
* 모호성 제거(Unambiguous) : 모호성 감소 기법(Model 및 그림, Formal requirements language, "shall")
* 일관성(Consistent)
* 검증성(Verifiable) : Inspection, Analysis, Demonstration, Test
* 가변성(Modifiable)
* 추적성(Traceable)


### 요구사항 명세에 포함될 내용

인터페이스, 기능, 성능, 속성, 설계 제약

### 요구사항 명세에 제외될 내용

프로젝트 요구사항(요구사항 명세서와 프로젝트 요구사항은 다른 생명 주기), Product 보증 계획, 설계 내용

