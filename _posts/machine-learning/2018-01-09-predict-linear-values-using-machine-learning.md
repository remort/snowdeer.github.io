---
layout: post
title: 선형 회귀 구현(Keras)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# Linear Regression

Keras를 이용해서 선형 회귀를 구현하는 예제입니다. 학습에 사용된 데이터는 [보스턴 집값](/assets/machine-learning/house_price.csv)입니다.

기존에 공부했던 코드들은 결과값이 0 또는 1, 또는 여러 범주 중 하나를 선택하는 코드들이었습니다. 이번 코드는 주어진 데이터를 이용해서 집값을 선형으로 예측하는 예제입니다.

0 또는 1의 값이 아니기 때문에 모델의 마지막 출력(output) 계층에서 활성화 함수(Activation Function)를 지정할 필요가 없습니다.

<br>

## 예제 코드

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense
from sklearn.model_selection import train_test_split

import pandas as pd

df = pd.read_csv('house_price.csv', delim_whitespace=True, header=None)

data_set = df.values
X = data_set[:, 0:13]
Y = data_set[:, 13]

X_train, X_validation, Y_train, Y_validation = train_test_split(X, Y,
                                                                test_size=0.2)

model = Sequential()
model.add(Dense(30, input_dim=13, activation='relu'))
model.add(Dense(6, activation='relu'))
model.add(Dense(1))

model.compile(loss='mean_squared_error', optimizer='adam')

model.fit(X_train, Y_train, epochs=200, batch_size=10)

Y_prediction = model.predict(X_validation).flatten()

for i in range(10):
  real_price = Y_validation[i]
  predicted_price = Y_prediction[i]
  print('Real Price: {:.3f}, Predicted Price: {:.3f}'.format(real_price,
                                                             predicted_price))
</pre>