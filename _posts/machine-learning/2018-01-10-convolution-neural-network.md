---
layout: post
title: CNN(Convolution Neural Network) 예제(Keras)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# CNN on Keras

Keras에서 CNN을 적용한 예제 코드입니다. MNIST 손글씨 데이터를 이용했으며, GPU 가속이 없는 상태에서는 수행 속도가 무척 느립니다.

<br>

## 예제 코드

<pre class="prettyprint">
from keras.datasets import mnist
from keras.utils import np_utils
from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPooling2D, Dropout, Flatten
from keras.callbacks import ModelCheckpoint, EarlyStopping

import matplotlib.pyplot as plt
import os
import numpy

MODEL_SAVE_FOLDER_PATH = './model/'

if not os.path.exists(MODEL_SAVE_FOLDER_PATH):
  os.mkdir(MODEL_SAVE_FOLDER_PATH)

model_path = MODEL_SAVE_FOLDER_PATH + 'mnist-' + '{epoch:02d}-{val_loss:.4f}.hdf5'

cb_checkpoint = ModelCheckpoint(filepath=model_path, monitor='val_loss',
                                verbose=1, save_best_only=True)

cb_early_stopping = EarlyStopping(monitor='val_loss', patience=10)

(X_train, Y_train), (X_validation, Y_validation) = mnist.load_data()

X_train = X_train.reshape(X_train.shape[0], 28, 28, 1).astype('float32') / 255
X_validation = X_validation.reshape(X_validation.shape[0], 28, 28, 1).astype('float32') / 255

Y_train = np_utils.to_categorical(Y_train, 10)
Y_validation = np_utils.to_categorical(Y_validation, 10)

model = Sequential()
model.add(Conv2D(32, kernel_size=(3, 3), input_shape=(28, 28, 1), activation='relu'))
model.add(Conv2D(64, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=2))
model.add(Dropout(0.25))
model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(10, activation='softmax'))

model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

history = model.fit(X_train, Y_train,
                    validation_data=(X_validation, Y_validation),
                    epochs=3, batch_size=200, verbose=0,
                    callbacks=[cb_checkpoint, cb_early_stopping])

print('\nAccuracy: {:.4f}'.format(model.evaluate(X_validation, Y_validation)[1]))

y_vloss = history.history['val_loss']
y_loss = history.history['loss']

x_len = numpy.arange(len(y_loss))
plt.plot(x_len, y_loss, marker='.', c='blue', label="Train-set Loss")
plt.plot(x_len, y_vloss, marker='.', c='red', label="Validation-set Loss")

plt.legend(loc='upper right')
plt.grid()
plt.xlabel('epoch')
plt.ylabel('loss')
plt.show()
</pre>

<br>

## 예제 코드 설명

Keras에서 Convolution Layer를 추가하는 코드는 다음과 같습니다.

<pre class="prettyprint">
model.add(Conv2D(32, kernel_size=(3, 3), input_shape=(28, 28, 1), activation='relu'))
</pre>

`Conv2D()` 함수의 인자로 

* 첫 번째 숫자인 `32`는 32개의 필터를 적용하겠다는 의미입니다.
* `kernel_size`는 필터의 크기를 의미합니다.
* `input_shape`는 (행, 열, 색상)을 의미합니다. 흑백의 경우 `1`의 값을 가집니다. RGB의 경우 `3`의 값을 가집니다.

<br>

<pre class="prettyprint">
model.add(MaxPooling2D(pool_size=2))
</pre>

`MaxPooling2D`는 풀링 기법 중 가장 많이 사용하는 맥스 풀링을 적용하는 함수입니다. 맥스 풀링은 정해진 영역 안에서 가장 큰 값만 남기고 나머지는 버리는 방식입니다. `pool_size`는 풀링 윈도우 크기이며 `2`의 값은 전체 크기를 절반으로 줄입니다.

<pre class="prettyprint">
model.add(Dropout(0.25))
</pre>

`Dropout()`는 특정 노드에 학습이 지나치게 몰리는 것을 방지하기 위해 랜덤하게 일부 노드를 꺼주는 역할을 합니다. Dropout을 통해 과적합을 조금 더 효과적으로 회피할 수 있습니다.

<pre class="prettyprint">
model.add(Flatten())
</pre>

지금까지 작업했던 이미지는 2차원 배열인데, `Flattern()` 함수를 통해 1차원 배열로 바꿔줄 수 있습니다.

<br>

## 실행 결과

![Image](/assets/machine-learning/035.png)

실행 결과입니다. 현재 사용하고 있는 노트PC의 GPU 성능이 낮아 GPU 가속을 받을 수 없는 상황이라 `epoch` 값을 3으로 했더니 위와 같은 결과가 나왔습니다. 환경이 좋은데서는 `epoch` 값을 더 늘려서 좀 더 성능이 좋은 모델을 얻을 수 있을 것입니다.