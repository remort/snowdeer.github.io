---
layout: post
title: [Keras] 폐암 수술 환자의 생존율 예측 예제
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# 폐암 수술 환자의 생존율 예측

Keras를 활용하여 폐암 수술 환자의 생존율을 예측하는 예제 코드입니다. 

예제 데이터는 [여기](/assets/machine-learning/thoraric_surgery.csv)에서 받을 수 있습니다.

<br>

# surgery.py

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense

import numpy

# numpy를 이용해서 csv 파일에서 데이터 불러오기
data_set = numpy.loadtxt('thoraric_surgery.csv', delimiter=',')

# X에는 0부터 17까지의 속성값
# Y에는 생존 여부(0 또는 1)
X = data_set[:, 0:17]
Y = data_set[:, 17]

# 계층 형태의 Model 생성
model = Sequential()

# Model에 계층 추가
model.add(Dense(10, input_dim=17, activation="relu"))
model.add(Dense(1, activation="sigmoid"))

# 모델 컴파일 및 학습
model.compile(loss='binary_crossentropy', optimizer='adam',
              metrics=['accuracy'])

model.fit(X, Y, epochs=50, batch_size=10)

# 결과 출력
print('\nAccuracy: {:.4f}'.format(model.evaluate(X, Y)[1]))
</pre>

<br>

# 결과 출력

~~~
...

 10/470 [..............................] - ETA: 0s - loss: 0.3850 - acc: 0.9000
420/470 [=========================>....] - ETA: 0s - loss: 0.4528 - acc: 0.8500
470/470 [==============================] - 0s 122us/step - loss: 0.4468 - acc: 0.8532
Epoch 47/50

 10/470 [..............................] - ETA: 0s - loss: 0.0977 - acc: 1.0000
420/470 [=========================>....] - ETA: 0s - loss: 0.4454 - acc: 0.8548
470/470 [==============================] - 0s 126us/step - loss: 0.4434 - acc: 0.8532
Epoch 48/50

 10/470 [..............................] - ETA: 0s - loss: 0.1985 - acc: 1.0000
470/470 [==============================] - 0s 96us/step - loss: 0.4628 - acc: 0.8511
Epoch 49/50

 10/470 [..............................] - ETA: 0s - loss: 0.3867 - acc: 0.9000
470/470 [==============================] - 0s 96us/step - loss: 0.4423 - acc: 0.8511
Epoch 50/50

 10/470 [..............................] - ETA: 0s - loss: 0.5494 - acc: 0.8000
270/470 [================>.............] - ETA: 0s - loss: 0.4599 - acc: 0.8593
470/470 [==============================] - 0s 213us/step - loss: 0.4498 - acc: 0.8532

 32/470 [=>............................] - ETA: 0s
470/470 [==============================] - 0s 49us/step

Accuracy: 0.8532
~~~