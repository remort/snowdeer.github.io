---
layout: post
title: 피마 인디언들의 당뇨병 예측(Keras)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# 피마 인디언들의 당뇨병 예측

Keras를 활용하여 피마 인디언들의 당뇨병 예측하는 예제 코드입니다. 

예제 데이터는 [여기]((/assets/machine-learning/pima-indians-diabetes.csv))에서 받을 수 있습니다.

<br>

# pima.py

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense

import numpy

data_set = numpy.loadtxt("pima-indians-diabetes.csv", delimiter=",")
X = data_set[:, 0:8]
Y = data_set[:, 8]

model = Sequential()

model.add(Dense(12, input_dim=8, activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy', optimizer='adam',
              metrics=['accuracy'])

model.fit(X, Y, epochs=100, batch_size=10)

print('\nAccuracy: {:.4f}'.format(model.evaluate(X, Y)[1]))
</pre>

<br>

# 결과 출력

~~~
...

 10/768 [..............................] - ETA: 0s - loss: 0.5410 - acc: 0.7000
540/768 [====================>.........] - ETA: 0s - loss: 0.5159 - acc: 0.7426
768/768 [==============================] - 0s 95us/step - loss: 0.5130 - acc: 0.7383
Epoch 98/100

 10/768 [..............................] - ETA: 0s - loss: 0.3326 - acc: 1.0000
470/768 [=================>............] - ETA: 0s - loss: 0.4856 - acc: 0.7681
768/768 [==============================] - 0s 104us/step - loss: 0.4867 - acc: 0.7682
Epoch 99/100

 10/768 [..............................] - ETA: 0s - loss: 0.6921 - acc: 0.6000
400/768 [==============>...............] - ETA: 0s - loss: 0.5100 - acc: 0.7625
768/768 [==============================] - 0s 132us/step - loss: 0.5060 - acc: 0.7539
Epoch 100/100

 10/768 [..............................] - ETA: 0s - loss: 0.7137 - acc: 0.6000
300/768 [==========>...................] - ETA: 0s - loss: 0.4650 - acc: 0.7733
620/768 [=======================>......] - ETA: 0s - loss: 0.4836 - acc: 0.7645
768/768 [==============================] - 0s 153us/step - loss: 0.4948 - acc: 0.7617

 32/768 [>.............................] - ETA: 0s
768/768 [==============================] - 0s 38us/step

Accuracy: 0.7721
~~~