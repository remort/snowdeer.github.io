---
layout: post
title: 시퀀스 다이어그램
category: UML
tag: [uml]
---

시퀀스 다이어그램(Class Diagram)은 참여자 사이의 시간 순서를 보여주는 다이어그램입니다.
시퀀스 다이어그램은 클래스 다이어그램과 함께 가장 많이 사용되는 UML입니다.

아래는 시퀀스 다이어그램의 간단한 예시입니다.

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/sequence.png)

[WebSequenceDiagrams](https://www.websequencediagrams.com/)와 같이 웹기반에서도
간편하게 다이어그램을 작성할 수 있는 Tool들도 많이 있습니다.

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/websequencediagrams.png)

<br> 

## 라이프라인(Lifeline)

시퀀스 다이어그램에서 세로로 길게 나열되어 있는 선을 라이프라인이라고 하며,
라이프 라인 위 사각형 형태로 각 인스턴스가 활성화(Activation)되어 있는 시간을 
표현하기도 합니다.

<br> 

## 메세지(Message)

메세지는 다음과 같은 형태가 있습니다.

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/message.png)

<br> 

## 분기문

분기문은 if문을 나타내는 opt와 if-then-else문을 나타내는 alt가 있습니다.

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/opt.png)

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/alt.png)

<br> 

## 반복문(Loop)

![Image]({{ site.baseurl }}/assets/2016-08-15-uml-sequence-diagram/loop.png)