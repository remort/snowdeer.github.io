---
layout: post
title: 퍼셉트론
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝]
---
# Perceptron

퍼셉트론은 1957년에 고안된 오래된 알고리즘이자, 딥러닝의 기원이 되는 알고리즘입니다.

퍼셉트론은 아래의 그림처럼 다수의 신호를 받아서 하나의 신호를 출력합니다. 이 때 출력값으로는 `0` 또는 `1`의 값만 가질 수 있습니다.

![Image](/assets/machine-learning/001.png) 

위 그림을 수식으로 표현하면

$$
output = w_1 x_1 + w_2 x_2 + ... w_i x_i
$$

가 되며, output은 0 또는 1의 값만 가지기 때문에 결과적으로 다음과 같은 식이 됩니다.

$$
output = 
\begin{cases}
0, if w_1 x_1 + w2 x_2 + ... + w_i x_i <= θ \\
1, if w_1 x_1 + w2 x_2 + ... + w_i x_i > θ
\end{cases}
$$

여기서 w 값은 가중치를 의미하며, `θ(theta)`는 임계값이라고 합니다.

<br>

## AND, OR, NAND 게이트

퍼셉트론을 이용하면 `AND`, `OR`, `NOR` 게이트 등을 아주 쉽게 만들 수 있습니다. 

예를 들면 다음과 같습니다.

<br>

### AND 게이트

x1 | x2 | y
---|---|---
0 | 0 | 0
1 | 0 | 0
0 | 1 | 0
1 | 1 | 1

이를 만족하는 (w1, w2, θ) 조합은 (0.5, 0.5, 0.7), (1.0, 1.0, 1.0) 등 무수히 많은 조합이 나올 수 있습니다.

<br>

### OR 게이트

