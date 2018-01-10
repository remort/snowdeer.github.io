---
layout: post
title: 아이리스 꽃 품종 예측하기(Keras, 다중 범주 분류)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# 아이리스 꽃 품종 예측

Keras를 활용하여 아이리스 꽃 품종 예측하는 예제 코드입니다. 

예제 데이터는 [여기]((/assets/machine-learning/iris.csv))에서 받을 수 있습니다.

<br>

# iris.py

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense
from keras.utils import np_utils
from sklearn.preprocessing import LabelEncoder

import pandas as pd

df = pd.read_csv('iris.csv',
                 names=["sepal_length", "sepal_width", "petal_length",
                        "petal_width", "species"])

data_set = df.values
X = data_set[:, 0:4].astype(float)
obj_y = data_set[:, 4]

encoder = LabelEncoder()
encoder.fit(obj_y)
Y_encodered = encoder.transform(obj_y)

Y = np_utils.to_categorical(Y_encodered)

model = Sequential()

model.add(Dense(16, input_dim=4, activation='relu'))
model.add(Dense(3, activation='softmax'))

model.compile(loss='categorical_crossentropy', optimizer='adam',
              metrics=['accuracy'])

model.fit(X, Y, epochs=100, batch_size=1)

print('\nAccuracy: %.4f' % (model.evaluate(X, Y)[1]))
</pre>

<br>

# 결과 출력

~~~
...

  1/150 [..............................] - ETA: 0s - loss: 8.4639e-06 - acc: 1.0000
 40/150 [=======>......................] - ETA: 0s - loss: 0.0309 - acc: 1.0000    
 76/150 [==============>...............] - ETA: 0s - loss: 0.0623 - acc: 0.9868
100/150 [===================>..........] - ETA: 0s - loss: 0.0760 - acc: 0.9800
128/150 [========================>.....] - ETA: 0s - loss: 0.0673 - acc: 0.9844
150/150 [==============================] - 0s 2ms/step - loss: 0.0638 - acc: 0.9867
Epoch 97/100

  1/150 [..............................] - ETA: 0s - loss: 7.2273e-04 - acc: 1.0000
 34/150 [=====>........................] - ETA: 0s - loss: 0.0260 - acc: 1.0000    
 75/150 [==============>...............] - ETA: 0s - loss: 0.0778 - acc: 0.9600
112/150 [=====================>........] - ETA: 0s - loss: 0.0857 - acc: 0.9643
148/150 [============================>.] - ETA: 0s - loss: 0.0762 - acc: 0.9730
150/150 [==============================] - 0s 1ms/step - loss: 0.0752 - acc: 0.9733
Epoch 98/100

  1/150 [..............................] - ETA: 0s - loss: 7.1290e-05 - acc: 1.0000
 29/150 [====>.........................] - ETA: 0s - loss: 0.0513 - acc: 1.0000    
 57/150 [==========>...................] - ETA: 0s - loss: 0.0419 - acc: 1.0000
 92/150 [=================>............] - ETA: 0s - loss: 0.0826 - acc: 0.9783
115/150 [======================>.......] - ETA: 0s - loss: 0.0699 - acc: 0.9826
141/150 [===========================>..] - ETA: 0s - loss: 0.0596 - acc: 0.9858
150/150 [==============================] - 0s 2ms/step - loss: 0.0750 - acc: 0.9800
Epoch 99/100

  1/150 [..............................] - ETA: 0s - loss: 0.2667 - acc: 1.0000
 46/150 [========>.....................] - ETA: 0s - loss: 0.1332 - acc: 0.9565
 90/150 [=================>............] - ETA: 0s - loss: 0.0866 - acc: 0.9778
126/150 [========================>.....] - ETA: 0s - loss: 0.0755 - acc: 0.9841
150/150 [==============================] - 0s 1ms/step - loss: 0.0726 - acc: 0.9800
Epoch 100/100

  1/150 [..............................] - ETA: 0s - loss: 0.0023 - acc: 1.0000
 53/150 [=========>....................] - ETA: 0s - loss: 0.0485 - acc: 0.9811
108/150 [====================>.........] - ETA: 0s - loss: 0.0768 - acc: 0.9815
150/150 [==============================] - 0s 956us/step - loss: 0.0717 - acc: 0.9800

 32/150 [=====>........................] - ETA: 0s
150/150 [==============================] - 0s 120us/step

Accuracy: 0.9800
~~~