---
layout: post
title: scanf vs cin
---

보통 사용자로부터 입력을 받을 때 C 언어에서는 scanf 함수를, C++ 에서는 cin을 많이 사용하는데
알고리즘 문제를 풀 때는 언어 상관없이 그냥 scanf 를 사용하는 것이 좋습니다.


[Algospot](https://algospot.com/)에 Input Method 별 수행 시간을 
측정해 놓은 [페이지](https://algospot.com/forum/read/2496/)가 있습니다. 
<br>
해당 페이지내의 실험 환경과 결과치를 보면, 

### 테스트 조건

* 입력: 10,000,000 개의 0~1023 범위의 정수
* 출력: 입력으로 받은 정수의 합
* 컴파일 환경 : g++ 4.8.2

### 수행 시간

* scanf : 0.798 (초)
* getchar : 0.390 (초)
* cin : 2.051 (초)

std::ios::sync_with_stdio(false) 옵션을 사용했을 때, 
cin은 scanf 와 비슷한 속도가 나오지만
추천하지는 않습니다. 

그냥 알고리즘 문제 풀 때는 깔끔하게 scanf 함수를 사용하도록 합시다.