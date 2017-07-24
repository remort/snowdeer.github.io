---
layout: post
title: Hyperledger 소개
category: BlockChain
tag: [BlockChain, Hyperledger]
---

## Hyperledger Project

![image -fullwidth]({{ site.baseurl }}/assets/2017-06-22-introduction-of-hyperledger/01.png)

하이퍼레저는 Linux Foundation으로 진행되고 있는 블록체인 오픈소스 중 하나입니다.
하이퍼레저 프로젝트의 창립 멤버였던 IBM이 44,000 라인의 블록체인 소스를 Hyperledger Fabric에
기부하면서 본격적인 프로젝트화가 진행되었습니다.

하이퍼레저는 다음과 같은 기능들을 제공합니다.
<ul>
 	<li>체인코드(Chaincode)를 통한 스마트 컨트랙트(Smart Contract)</li>
 	<li>디지털 자산(Digital Assets)</li>
 	<li>저장 시스템 &amp; 기록 보관소</li>
 	<li>분산 합의 네트워크</li>
 	<li>플러그인 형태의 합의(Consensus) 알고리즘</li>
</ul>

<br>
## 하이퍼레저 구조
하이퍼레저는 다음과 같은 구조로 되어 있습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-06-22-introduction-of-hyperledger/02.png)

크게 Membership, Blockchain, Chaincode로 이루어져 있다고 생각할 수 있습니다.
Membership을 통해 가입이나 참여자의 신원 확인 등을 할 수 있고, Blockchain은 HTTP/2 기반의
P2P 분산 장부 프로토콜이 구현되어 있습니다. Chaincode 부분은 일반적인 스마트 컨트랙트 부분이라고
생각하시면 되고 여기에서 다양한 서비스들이 응용되어질 수 있습니다.

<br>

## Hyperledger White Paper
[Hyperledger Whitepaper](www.the-blockchain.com/docs/Hyperledger%20Whitepaper.pdf)

하이퍼레저 백서는 20페이지 가량으로 간단하게 소개를 하고 있어 읽어볼만한 문서입니다.

<br>

## 하이퍼레저 프로젝트들
하이퍼레저는 크게 다음과 같은 프로젝트들로 이루어져 있습니다.
<ul>
 	<li>Blockchain Explorer</li>
 	<li>Fabric</li>
 	<li>Iroha</li>
 	<li>Sawtooth Lake</li>
</ul>
Blockchain Explorer는 블록체인 런타임의 다양한 정보를 모니터링하기 위한
웹기반 어플리케이션을 개발하는 프로젝트입니다.

Fabric는 블록체인 엔진을 다루고 있는 프로젝트이며, 가장 중심이 되는 프로젝트라고
볼 수 있습니다. 최근에 0.6에서 1.0으로 버전업이 되었습니다.

Iroha는 기존 시스템에 블록체인의 분산 원장을 쉽게 도입시키려는 목적을 가진 프로젝트입니다.
대표적으로 다음과 같은 특징을 갖고 있습니다.
<ul>
 	<li>기존 시스템에 쉽게 연동</li>
 	<li>최신 도메인 지향 C++ 디자인</li>
 	<li>Mobile Application 지원 강화</li>
 	<li>비잔틴 장애 허용 알고리즘(Sumeragi) 채택</li>
</ul>

Sawtooth Lake는 인텔(Intel)의 모듈라 방식의 블록체인 제품군으로 기존 산업에
블록체인을 활용할 수 있도록 하기위한 프로젝트입니다.

<br>
## Hyperledger Fabric
위 프로젝트들 중에서 Fabric는 하이퍼레저의 핵심 프로젝트로 블록체인 엔진을 다루고 있습니다.
[문서화](http://hyperledger-fabric.readthedocs.io/en/latest/?cm_mc_uid=39188216865714963167367&amp;cm_mc_sid_50200000=1498110162)도 상당히 잘되어 있으며, 최근에 1.0 버전으로 업데이트 했습니다.

1.0 버전이 되면서 개인 인증 부분에 아주 큰 변화가 있었고, 사용자별로 인증할 수 있는
기능이 추가되었습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-06-22-introduction-of-hyperledger/03.png)
