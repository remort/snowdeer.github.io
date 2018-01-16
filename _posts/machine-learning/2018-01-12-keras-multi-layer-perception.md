---
layout: post
title: Keras - 다층 퍼셉트론(MLP, Multi-Layer Perception) 
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# Keras 다층 퍼셉트론 구현

Keras에서 다층 퍼셉트론은 다음과 같은 코드를 이용해서 구현합니다.

<pre class="prettyprint">
model = Sequential()
model.add(Dense(12, input_dim=8, activation='relu'))
model.add(Dense(8, activation='relu))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
</pre>

첫 번째 라인의 `Sequential()`으로 다층 퍼셉트론 형태를 만들겠다는 의미이며, 그 다음 라인의 `Dense()`들이 각 계층을 의미합니다. 또한 마지막에 `compile()` 과정을 거쳐 모델을 생성합니다.

<br>

## Dense 파라메터

`Dense()` 계층을 생성할 때 각 파라메터는 다음과 같은 의미를 가집니다.

~~~
model.add(Dense(12, input_dim=8, activation='relu'))
~~~

맨 앞의 숫자 `12`는 해당 계층의 노드 개수입니다. `input_dim=8`의 경우 입력단에 8개의 데이터 항목이 존재한다는 의미이며, 보통 첫 번째 Layer에서만 `input_dim` 파라메터를 입력합니다. `activation`은 활성화 함수를 의미합니다.

활성화 함수는 다음과 같은 종류를 사용할 수 있습니다.

* linear : 기본값이며, 가중치 결과값이 출력으로 그대로 나옵니다.
* relu : ReLU 함수입니다. 은닉층에서 주로 사용합니다.
* sigmoid : 주로 출력층에서 사용합니다.
* softmax : 출력값들의 합이 `1.0`이 되도록 하는 함수로 보통 출력층에서 사용합니다.

<br>

## compile 옵션

`compile()` 함수의 옵션은 각각 다음과 같습니다.

* loss : 현재 가중치 세트를 평가하는데 사용하는 손실 함수입니다.
* optimizer : 최적의 가중치를 찾는 최적화 알고리즘으로, `adam`은 효율적인 경사 하강법 알고리즘 중 하나입니다.
* metrics :  평가 척도를 의미하며, 일반적으로 `accuracy`를 사용합니다.