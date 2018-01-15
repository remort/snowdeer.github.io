---
layout: post
title: [Keras] History 기능 사용하기
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# Keras 학습 이력 기능

Keras에서는 모델 학습을 위해 `fit()` 함수를 사용합니다. 이 때, 리턴값으로 학습 이력(History) 정보를 리턴합니다. 여기에는 다음과 같은 항목들이 포함되어 있습니다.

아래 항목들은 매 epoch 마다의 값들이 저장되어 있습니다.

* loss : 훈련 손실값
* acc : 훈련 정확도
* val_loss : 검증 손실값
* val_acc : 검증 정확도

<br>

## 학습 이력 확인

학습이 끝난 후 다음 코드로 쉽게 확인이 가능합니다.

<pre class="prettyprint">
hist = model.fit(X_train, Y_train, validation_data=(X_validation, Y_validation),
          epochs=30, batch_size=500)

print(hist.history['loss'])
print(hist.history['acc'])
print(hist.history['val_loss'])
print(hist.history['val_acc'])
</pre>

<br>

## 학습 이력 그래프로 확인

`matplotlib`의 `pyplot`를 이용해서 각 결과를 그래프로 조회할 수 있습니다.

<pre class="prettyprint">
from keras.datasets import mnist
from keras.utils import np_utils
from keras.models import Sequential
from keras.layers import Dense

(X_train, Y_train), (X_validation, Y_validation) = mnist.load_data()

X_train = X_train.reshape(X_train.shape[0], 784).astype('float64') / 255
X_validation = X_validation.reshape(X_validation.shape[0], 784).astype('float64') / 255

Y_train = np_utils.to_categorical(Y_train, 10)
Y_validation = np_utils.to_categorical(Y_validation, 10)

model = Sequential()
model.add(Dense(512, input_dim=784, activation='relu'))
model.add(Dense(10, activation='softmax'))

model.compile(loss='categorical_crossentropy', optimizer='adam',
              metrics=['accuracy'])

hist = model.fit(X_train, Y_train, validation_data=(X_validation, Y_validation),
          epochs=30, batch_size=500)

print('\nAccuracy: {:.4f}'.format(model.evaluate(X_validation, Y_validation)[1]))


import matplotlib.pyplot as plt

fig, loss_ax = plt.subplots()
acc_ax = loss_ax.twinx()

loss_ax.plot(hist.history['loss'], 'y', label='train loss')
loss_ax.plot(hist.history['val_loss'], 'r', label='val loss')
loss_ax.set_xlabel('epoch')
loss_ax.set_ylabel('loss')
loss_ax.legend(loc='upper left')

acc_ax.plot(hist.history['acc'], 'b', label='train acc')
acc_ax.plot(hist.history['val_acc'], 'g', label='val acc')
acc_ax.set_ylabel('accuracy')
acc_ax.legend(loc='upper left')

plt.show()
</pre>

![Image](/assets/machine-learning/036.png)

위와 같이 그래프로 각각의 항목들의 추이를 확인할 수 있습니다. 사실 위 경우는 학습 속도 조절을 위해 `epochs` 값과 `batch_size` 값을 임의로 조정해놓아서 과적합이 발생하는 시점을 확인할 수 없지만, 보통 `epochs` 값이 지나치게 클 수록 과적합이 발생하여 실제 검증 정확도 `val_acc`는 점점 하락하는 것을 확인할 수 있습니다.
