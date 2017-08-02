---
layout: post
title: SW 품질 속성 6가지
category: S/W Architecture
permalink: /sw-architecture/:year/:month/:day/:title/

tag: [품질]
---

품질 속성(QA : Quality Attribute)은 시스템이 이해 당사자의 요구 사항을 얼마나 잘 만족시키는지를
나타내기 위해 사용하며, 측정하거나 테스트할 수 있는 시스템의 특성을 의미합니다.

품질 속성은 보통 다음과 같이 6개의 요소로 구성됩니다.

![image](/assets/2017-02-15-sw-quality-attribute/01.jpg)

<br>

# Stmulus(자극)

자극은 시스템에 도달하는 이벤트를 말합니다. 사용자의 명령이 될 수도 있고, 보안적인 측면에서는 외부로부터의 공격이 될 수도 있습니다.

<br>

# Source of Stimulus(자극원)

자극원은 자극의 원인입니다. 자극원은 시스템이 처리하는 방식에 영향을 줄 수 있습니다. 예를 들면, 신뢰할 수 있는 사용자로부터의 요청과 신뢰할 수 없는 사용자로부터의 요청은 서로 다르게 수행될 것입니다.

<br>

# Response(반응)

시스템이 자극에 반응하는 방법입니다.

<br>

# Response Measure(반응 측정)

반응에 대한 측정입니다. 반응이 요구 사항을 만족시켰는지 여부를 결정하며, Latency(지연 시간)나 Throughput(산출량) 등이 측정 요소가 될 수 있습니다.

<br>

# Environment(환경)

시나리오가 발생하는 상황입니다. 자극은 특정한 조건에서 발생하며 환경은 자극을 가하는 역할을 합니다.

<br>

# Artifact(대상)

대상은 시스템을 말합니다. 전체 시스템이 될 수도 있고 특정 부분이 될 수도 있습니다. 예를 들면 데이터 저장소(Data Store)와 메타 데이터 저장소(Metadata Store)에서의 오류는 다르게 처리되어야 할 것입니다.

## 예제

가용성이라는 시나리오 기반으로 각 요소들을 생각하면 다음과 같습니다.

![image](/assets/2017-02-15-sw-quality-attribute/02.png)
