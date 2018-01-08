---
layout: post
title: 신경망
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---
# Neural Network

[퍼셉트론](machine-learning/2018/01/02/perceptron/)을 이용하면 복잡하고 어려운 함수도 표현이 가능합니다. 하지만, 가중치를 설정하는 작업을 사람이 수동으로 해줘야하는 불편함이 여전히 존재합니다.

신경망(Neural Network)은 이러한 문제점을 해결해줍니다. 가중치를 학습을 통해서 자동으로 획득하고 수정할 수 있습니다.

<br>

# Simple Neural Network

신경망을 간단하게 그림으로 표현하면 다음과 같습니다.

![Image](/assets/machine-learning/004.png) 

데이터가 입력되는 쪽을 `Input`, 결과값이 출력되는 쪽을 `Output`, 그리고 그 사이를 `Hidden`이라고 합니다. 층이 많아지면 아래의 그림과 같은 모습이 됩니다.

![Image](/assets/machine-learning/005.png) 

<br>

# 퍼셉트론과 신경망

신경망은 퍼셉트론과 크게 다르지 않습니다. 퍼셉트론은 다음과 같은 함수로 이루어져 있습니다.

$$
output = 
\begin{cases}
0 ( w_1 x_1 + w2 x_2 + ... + w_i x_i \le 0 ) \\
1 ( w_1 x_1 + w2 x_2 + ... + w_i x_i > 0 )
\end{cases}
$$

이걸 좀 더 간결하게 표현하면 다음과 같이 표현할 수 있습니다.

$$
y = h(b + w_1 x_1 + w_2 + x_2 + ... w_i x_i)
h(x) = 
\begin{cases}
0 ( x \le 0 ) \\
1 ( x> 0 )
\end{cases}
$$

입력 신호의 총합이 `h(x)`라는 함수를 거쳐서 변환이 되며, 그 변환된 값이 y 출력값이 됩니다.

<br>

# Activation Function

위에서 나온 `h(x)` 함수는 활성화 함수(Activation Function)라고 합니다. 입력 신호의 총합을 출력 신호로 변환해주는 함수입니다.

위의 수식에서 $ b + w_1 x_1 + w_2 + x_2 + ... w_i x_i $ 부분을 `a`라는 변수로 치환하면 다음과 같은 식으로 표현이 됩니다.

$$
a = b + w_1 x_1 + w_2 + x_2 + ... w_i x_i
y = h(a)
$$

그림으로 표현하면 아래와 같은 그림이 되고,

![Image](/assets/machine-learning/006.jpg) 

조금 더 간결하게 표현하면 다음과 같이 표현됩니다.

![Image](/assets/machine-learning/007.png) 

