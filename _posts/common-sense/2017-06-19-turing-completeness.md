---
layout: post
title: 튜링 완전(Turing Completeness)
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [용어]
---

튜링 완전(Turing Completeness)라는 단어가 있습니다. 형용사적인 표현으로 Turing Complete라는 표현을 쓰기도 하는데, 어떤 뜻이 있는지 포스팅해보도록 하겠습니다.

<br>

# 튜링 머신

1936년 수학자 앨런 튜링(Alan Mathison Turing)이라는 사람이 제시한 개념입니다. 계산하는 기계라는 설명하기 위한 단어로, 원래는 Automatic에서 따온 a-machine이라고 명칭했지만, 앨런 튜링 사망 이후 창시자의 이름을 따서 '튜링 머신'이라고 부르기 시작했습니다.

튜링머신은 다음과 같은 구성 요소로 이루어져 있습니다.

<ul>
 	<li>테이프(Tape) : 일정한 크기의 셀(Cell)로 이루어져 있으며, 길이는 무한</li>
 	<li>헤드(Head) : 테이프를 읽어들이는 헤드, 테이프의 위치간 이동이 가능</li>
 	<li>상태 기록기(State register) : 현재 튜링 머신의 상태를 기록하는 장치</li>
 	<li>행동표(Action Table) : Instruction Table이라고도 하며, 특정 조건에서 수행해야 할 행동을 기록함</li>
</ul>

<br>

대략적인 예시를 들면 다음과 같습니다.

<ul>
 	<li>현재 상태가 'A'인데 테이프에서 '1'이라는 값을 읽으면 상태를 'C'로 하고 테이프 한 칸 전진</li>
 	<li>현재 상태가 'B'인데 '100'이라는 값을 읽으면 상태를 그대로 하고, 테이프 한 칸 전진</li>
</ul>

<br>

# 현재의 컴퓨터는 Turing Complete

현재의 컴퓨터들은 대부분 Turing Complete하다고 할 수 있습니다. 하지만, 엄밀히 따지면 보통 메모리의 한계 등이 있기 때문에 Turing Completeness에 가깝다고 하는 것이 더 정확할 것 같습니다. 과거에는 Turing Complete하지 않은 컴퓨터도 있었다고 합니다.

<br>

# 프로그래밍 언어

Turing Completeness는 오히여 프로그래밍 언어에서 더 많이 사용됩니다. 하지만, 우리가 자주 사용하는 대표적인 언어들인 C, Java, Python 등은 Turing Complete한 언어입니다. 보통 대표적으로 다음 기능을 갖고 있으면 Turing Complete하다고 합니다.
<ul>
 	<li>Conditional Branch : 조건 분기문을 가질 수 있어야 합니다. IF 문 또는 FOR, WHILE 등의 반복문 또한 여기에 해당합니다.</li>
 	<li>메모리의 임의 위치의 값을 변경할 수 있어야 한다.</li>
</ul>
이 두가지 기능을 갖고 있으면, 우리가 사용하는 거의 모든 기능들을 위 기능의 조합으로 만들어낼 수 있다고 합니다.

대부분의 언어들은 Turing Complete한 성격을 갖고 있습니다. 그럼 Turing Complete하지 않은 언어들은 무엇이 있을까요? 특수한 목적을 갖고 있는 스크립트(Script)언어들이 여기에 해당합니다. HTML이나 SQL, XML 등의 언어를 예로 들 수 있습니다.
