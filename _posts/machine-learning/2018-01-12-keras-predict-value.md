---
layout: post
title: Keras - 수치 예측하기 예제
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# Keras를 이용한 수치 예측하기 예제

다음과 같은 랜덤 데이터셋을 만들었을 때, 그 데이터를 학습한 다음 향후 들어오는 입력값을 예측하는 예제코드입니다.

<pre class="prettyprint">
import numpy as np

x_train = np.random.random((1000, 1))
y_train = x_train * 2 + np.random.random((1000, 1)) / 3.0

x_test = np.random.random((100, 1))
y_test = x_test * 2 + np.random.random((100, 1)) / 3.0

import matplotlib.pyplot as plt

plt.plot(x_train, y_train, 'ro')
plt.plot(x_test, y_test, 'bo')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
</pre>

![Image](/assets/machine-learning/037.png)

입력값 X에 2를 곱해서 Y 값을 만들어내는 식이기 때문에 `Y = w * X + b' 형태의 수식으로 표현할 수 있으며, w는 2에 가깝고, b는 0.16에 가까워지면 정답에 근접해집니다.

<br>

## 일반적인 선형회귀 모델(numpy 활용)

<pre class="prettyprint">
import numpy as np
from sklearn.metrics import mean_squared_error

x_train = np.random.random((1000, 1))
y_train = x_train * 2 + np.random.random((1000, 1)) / 3.0

x_test = np.random.random((100, 1))
y_test = x_test * 2 + np.random.random((100, 1)) / 3.0

x_train = x_train.reshape(1000, )
y_train = y_train.reshape(1000, )
x_test = x_test.reshape(100, )
y_test = y_test.reshape(100, )

w = np.cov(x_train, y_train, bias=1)[0, 1] / np.var(x_train)
b = np.average(y_train) - w * np.average(x_train)

print('w:{0}, b:{1}'.format(w, b))

y_predict = w * x_test + b
mse = mean_squared_error(y_test, y_predict)
print('mse: {}'.format(mse))
</pre>

<br>

## 퍼셉트론 신경망 모델

<pre class="prettyprint">
import numpy as np
from keras.models import Sequential
from keras.layers import Dense

x_train = np.random.random((1000, 1))
y_train = x_train * 2 + np.random.random((1000, 1)) / 3.0

x_test = np.random.random((100, 1))
y_test = x_test * 2 + np.random.random((100, 1)) / 3.0

model = Sequential()
model.add(Dense(1, input_dim=1))

model.compile(optimizer='rmsprop', loss='mse')

hist = model.fit(x_train, y_train, epochs=50, batch_size=10)
w, b = model.get_weights()
print('w:{0}, b:{1}'.format(w, b))

import matplotlib.pyplot as plt

plt.plot(hist.history['loss'])
plt.ylim(0.0, 1.5)
plt.xlabel('epoch')
plt.ylabel('loss')
plt.legend(['train'], loc='upper left')
plt.show()

loss = model.evaluate(x_test, y_test, batch_size=10)
print('loss: {0}'.format(loss))
</pre>

결과는 다음과 같습니다.

~~~
w:[[2.009994]], b:[0.16119921]
...
loss: 0.010309214843437076
~~~

![Image](/assets/machine-learning/038.png)

<br>

## 다층 퍼셉트론 신경망 모델

<pre class="prettyprint">
import numpy as np
from keras.models import Sequential
from keras.layers import Dense

x_train = np.random.random((1000, 1))
y_train = x_train * 2 + np.random.random((1000, 1)) / 3.0

x_test = np.random.random((100, 1))
y_test = x_test * 2 + np.random.random((100, 1)) / 3.0

model = Sequential()
model.add(Dense(64, input_dim=1, activation='relu'))
model.add(Dense(1))

model.compile(optimizer='rmsprop', loss='mse')

hist = model.fit(x_train, y_train, epochs=50, batch_size=10)

import matplotlib.pyplot as plt

plt.plot(hist.history['loss'])
plt.ylim(0.0, 1.5)
plt.xlabel('epoch')
plt.ylabel('loss')
plt.legend(['train'], loc='upper left')
plt.show()

loss = model.evaluate(x_test, y_test, batch_size=10)
print('loss: {0}'.format(loss))
</pre>

결과입니다.

~~~
...
loss: 0.008995027653872967
~~~

![Image](/assets/machine-learning/039.png)