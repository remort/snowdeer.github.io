---
layout: post
title: 머신 러닝(Machine Learning)
category: 개발상식
permalink: /common-sense/:year/:month/:day/:title/

tag: [머신러닝]
---

~~~
'머신 러닝(Machine Learning)'은 사람이 컴퓨터에게 직접 명령을 내리는 것이 아니라 컴퓨터가 스스로 다수의 데이터를 이용하여 학습을 하고, 자동으로 문제를 풀어나가는 기술을 말합니다.
~~~

[Machine Learning](https://ko.wikipedia.org/wiki/%EB%A8%B8%EC%8B%A0_%EB%9F%AC%EB%8B%9D)는 구글의 [알파고](https://ko.wikipedia.org/wiki/%EC%95%8C%ED%8C%8C%EA%B3%A0) 로 인해서 갑자기 더 유명해졌습니다.

Machine Learning에 대해서 구체적인 예를 들어보도록 하겠습니다.

메일에서 스팸(Spam) 메일을 구분하는 방법입니다.

* 제일 쉬운 방법은 블랙 리스트를 관리하는 방법입니다.
* 하지만 메일을 보낸 사람의 이름을 바꿔버리면 ?
* 그 다음으로 쉬운 방법은 제목에서 특정 단어들을 찾는 방법입니다.
* 하지만 메일 제목을 '광.고*'와 같은 식으로 바꿔버리면 ?

이제부터는 단순히 룰(Rule)기반 알고리즘만으로는 스팸 메일을 구분하기 힘들어집니다. 하지만, Machine Learning을 적용해서 좀 더 효율적으로 처리할 수 있습니다. 

먼저 컴퓨터에게 정상 메일과 스팸 메일들을 잔뜩 주고나서, 학습을 하도록 합니다. 그러면 컴퓨터가 스스로 메일들을 분류하고 학습해가며 스팸 메일을 찾아낼 수 있게 됩니다.  
(물론 100% 정답을 찾아내는 것은 어렵습니다. 대신 데이터가 많으면 많을 수록, 학습을 많이 하면 할 수록 정답에 가까운 확률로 스팸 메일을 찾아낼 수 있게 될 것입니다.)

위의 예제와 같이 Machine Learning은 Rule 기반의 동작과는 거리가 좀 있습니다.


보통 Machine Learning과 [데이터 마이닝(Data Mining)](https://ko.wikipedia.org/wiki/%EB%8D%B0%EC%9D%B4%ED%84%B0_%EB%A7%88%EC%9D%B4%EB%8B%9D)이 같은 개념으로 
사용되기도 하는데, 사실은 조금 다른 특징이 있습니다.

* Machine Learning는 대규모의 Training Data를 기반으로 학습을 하여 예측에 중점을 두고 있습니다.
* Data Mining는 이름 그대로 Data를 발굴하는 것입니다. 미처 몰랐던 정보를 찾아내는 것에 중점을 두고 있습니다.

Machine Learning은 다양한 알고리즘들이 쓰이며,  최고의 알고리즘은 없습니다. 다만, 용도에 따라 더 효율적인 알고리즘은 존재할 수 있습니다.

Machine Learning를 간단한게 분류지어보면 다음과 같습니다.

<br>

# Taxonomy of Machine Learning

![Image](/assets/2016-09-03-machine-learning-algorithms/slide_15.jpg) 

다음 번에는 Machine Learning에 대한 대표적인 알고리즘들에 대해서 간단히 포스팅 하도록 하겠습니다.