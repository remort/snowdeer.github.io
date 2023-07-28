---
layout: post
title: Microservice Architecture, Doocker, Kubernetes
category: Kubernetes
permalink: /kubernetes/:year/:month/:day/:title/

tag: [Kubernetes, k8s]
---

# Microservice Architecture

조대협님의 [Youtube 강좌](https://www.youtube.com/watch?v=xdqOxF2JqwU)를 보면서 나름대로 요약한 글입니다. 또한 SlideShare는 [여기](https://www.slideshare.net/Byungwook/micro-service-architecture-52233912)를 참고하면 됩니다.

## 모노리틱 아키텍처(Monolithic Architecture)

모노리틱 아키텍처(Monolithic Architecture)는 예전부터 사용되어져오던 아키텍처이며, 다음 이미지와 같이 하나의 통서버에서 모든 것들을 처리하는 시스템 구조입니다.

![Image](/assets/kubernetes/001.png)

하나의 서버에 모든 비즈니스 로직이 포함되며, 하나의 중앙 집중형 데이터베이스에 모든 데이터가 저장됩니다. 다양한 기술들을 혼용해서 사용하기 어렵기 때문에 단일화된 기술을 주로 사용합니다.

장점으로는

- 단일화된 기술 사용
- 관리가 수월함

이 있으며, 단점으로는

- 여러 기술 혼용이 어렵기 때문에 부위별 적절한 기술 사용이 어려움
- 배포 및 재기동 시간이 오래 걸림
- 향후 수정이 용이하지 않음

가 있습니다.

<br>

## 마이크로서비스 아키텍처(Microservice Architecture)

마이크로서비스 아키텍처(Microservice Architecture)는 시스템을 여러 개의 독립된 서비스로 분할하고 조합하여 시스템을 구성하는 방법입니다. 과거 SOA(Service Oriented Architecture)의 경량화된 버전이라고 볼 수 있습니다.

여러 개의 서비스들로 이루어져 있으며, 각각은 RESTful API 등을 이용해서 통신을 합니다. 서비스별로 다른 기술 스택을 사용할 수 있기 때문에 적재적소에 알맞은 기술을 선택할 수 있는 장점이 있습니다.

장점으로는

- 필요한 서비스에 적절한 기술을 적용 가능 (ex. 복잡한 데이터는 RDBMS를 사용하고, 양이 많은 고속 데이터는 NoSQL을 사용한다던지, 빠른 개발로 스크립트 언어를 사용하고 튼튼한 시스템 개발로는 Java를 사용하는 등)

이 있으며, 단점으로는

- 여러 기술을 동시에 다뤄야 하기 때문에 운영의 부담이 커질 수 있음
- 따라서 개발자가 떠나면 유지 보수가 어려움

과 같은 단점이 있습니다.

![Image](/assets/kubernetes/002.jpg)

<br>

## 마이크로 서비스 아키텍처 구성

마이크로 서비스 아키텍처 구조는 보통 다음 그림과 같습니다.

![Image](/assets/kubernetes/003.png)

<br>

### API 게이트웨이

API 게이트웨이는 클라이언트와 API 서버 앞에서 일종의 프록시 역할을 담당하며 다음과 같은 역할을 합니다.

- API 인증/인가
- Logging
- Routing
- 메시지 변환
- 메시지 프로토콜 변환

API 게이트웨이는 SOA ESB(Enterprise Service BUS)의 단순화 버전이라고 볼 수 있으며 필수적인 요소는 아닙니다. 잘 사용하면 좋은 컴포넌트가 될 수 있지만, 잘 못 쓰면 서비스를 망칠 수도 있기 때문에 도입은 신중하게 할 필요가 있습니다.

<br>

### API 게이트웨이를 이용한 설계 패턴

API 게이트웨이를 이용한 설계 패턴들은 다음과 같습니다.

![Image](/assets/kubernetes/004.png)

![Image](/assets/kubernetes/005.png)

![Image](/assets/kubernetes/006.png)

![Image](/assets/kubernetes/007.png)

![Image](/assets/kubernetes/008.png)

![Image](/assets/kubernetes/009.png)

![Image](/assets/kubernetes/010.png)

![Image](/assets/kubernetes/011.png)

![Image](/assets/kubernetes/012.png)

![Image](/assets/kubernetes/013.png)

<br>

# Docker

Docker의 기본 설명은 생략을 하도록 하겠습니다. Docker를 사용하다보면 컨테이너들을 어떤 물리 서버에 배포할 것인지가 이슈가 됩니다. 또한 컨테이너 관리나(Container Management), 로드 밸런싱(Load Balancing), 헬스 체크(Health Check), Rolling Upgrade 등의 이슈도 같이 발생합니다.

그래서 Docker Swarm, Apache Mesos, Kubernetes 등의 솔루션이 있습니다.

이 중에서 Kubernetes는 다음과 같습니다.

<br>

# Kubernetes

구글은 과거부터 컨테이너 서비스를 운영해왔습니다. VM보다 컨테이너를 더 오래 사용했으며, 현재 20억개 이상의 컨테이너를 운영하고 있습니다.

Kubernetes는 구글의 오랜 컨테이너 서비스 운영 경험의 산물이라고 볼 수 있습니다. Public, Private 인프라에서 모두 사용가능한 오픈 소스입니다.

<br>

## Pods

Pod는 Kubernetes의 최소 논리 단위이며 하나의 어플을 표현하는 최소 단위이기도 합니다. 하나의 Pod에는 복수 개의 컨테이너가 포함될 수 있으며, 주로 Tightly Coupled 되는 컨테이너들을 하나의 Pod에 묶어서 사용합니다. 예를 들면 Nginx와 Tomcat를 묶는다던지 Tomcat과 Memcached를 묶을 수 있습니다.

Pod에 있는 컨테이너들은 물리적으로 같은 서버에 생성이 되며, 같은 Pod에 있는 컨테이너들끼리는 Disk Volume을 공유할 수 있습니다.

![Image](/assets/kubernetes/014.png)

<br>

## Replication Controllers

Replication Controller는 Pod들을 생성하고 관리하는 역할을 합니다. Pod 생성은 미리 정의된 Template로부터 생성되며 Pod의 가동상태를 체크하고, 죽으면 다시 기동시키는 역할, Auto Scaling 등의 작업을 합니다.

![Image](/assets/kubernetes/015.png)

<br>

## Services

서비스는 Pod 들의 집합(ex. 웹서버 서비스, 백엔드 서비스, 캐쉬 서비스)이며 IP Address가 지정(Expose)되고, Pod간 로드 밸런싱 제공합니다.

![Image](/assets/kubernetes/016.png)
