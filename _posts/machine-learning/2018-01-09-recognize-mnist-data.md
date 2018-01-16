---
layout: post
title: Keras - MNIST 손글씨 인식하기
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# MNIST

MNIST 손글씨 인식은 머신러닝의 'Hello, World'라고 불리울정도로 기본이 되는 예제입니다. 

![Image](/assets/machine-learning/024.png)

미국 국립표준기술원(NIST)에서 고등학생과 인구조사국 직원 등이 쓴 손글씨를 수집하여 70,000개의 숫자 손글씨 데이터셋으로 만들었습니다.

MNIST 데이터는 이미지 형태의 데이터다보니 머신러닝의 입력 데이터로 변환하는 전처리 작업을 해줘야 합니다.

<br>

## 다운로드 및 전처리

MNIST 데이터는 워낙 유명하다보니, Keras에서 기본적으로 쉽게 불러올 수 있는 기능을 제공하고 있습니다.

MNIST 데이터는 학습용 데이터 60,000개, 검증용 데이터 10,000개로 이루어져 있습니다.

<pre class="prettyprint">
from keras.datasets import mnist

(X_train, Y_train), (X_validation, Y_validation) = mnist.load_data()
</pre>

위 코드로 MNIST 데이터를 네트워크에서 다운받아서 각각의 변수에 불러오도록 수행합니다. 다운로드는 처음에 한 번만 수행하며, 그 뒤로는 이미 다운받은 데이터를 활용해서 불러오기를 수행합니다.

<br>

MNIST 데이터는 이미지로 되어 있어서 컴퓨터에서 인식할 수 있도록 전처리 작업을 해주어야 합니다. 손글씨 한 장의 이미지는 28 x 28 = 784개의 픽셀로 이루어져 있습니다. 

아래 코드로 픽셀 정보를 눈으로 확인할 수 있습니다.

<pre class="prettyprint">
(X_train, Y_train), (X_validation, Y_validation) = mnist.load_data()

for x in X_train[0]:
  for i in x:
    print('{:3} '.format(i), end='')
  print()
</pre>

픽셀 정보는 다음과 같습니다.

~~~
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   3  18  18  18 126 136 175  26 166 255 247 127   0   
  0   0   0   0   0   0  30  36  94 154 170 253 253 253 253 253 225 172 253 242 195  64   0  
  0   0   0   0   0  49 238 253 253 253 253 253 253 253 253 251  93  82  82  56  39   0   0   
  0   0   0   0   0  18 219 253 253 253 253 253 198 182 247 241   0   0   0   0   0   0   0  
  0   0   0   0   0   0  80 156 107 253 253 205  11   0  43 154   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0  14   1 154 253  90   0   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0 139 253 190   2   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0  11 190 253  70   0   0   0   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0  35 241 225 160 108   1   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0   0  81 240 253 253 119  25   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0  45 186 253 253 150  27   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0   0  16  93 252 253 187   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0 249 253 249  64   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0  46 130 183 253 253 207   2   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0  39 148 229 253 253 253 250 182   0   0   0   0   0   
  0   0   0   0   0   0   0   0  24 114 221 253 253 253 253 201  78   0   0   0   0   0   0   
  0   0   0   0   0   0  23  66 213 253 253 253 253 198  81   2   0   0   0   0   0   0   0   
  0   0   0   0  18 171 219 253 253 253 253 195  80   9   0   0   0   0   0   0   0   0   0   
  0   0  55 172 226 253 253 253 253 244 133  11   0   0   0   0   0   0   0   0   0   0   0    
  0   0 136 253 253 253 212 135 132  16   0   0   0   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0  
~~~

<br>

손글씨 정보를 다음과 같은 순서로 변환해줍니다.

* 가로28, 세로28개의 배열을 1차원 784개의 배열로 변환
* 각 픽셀당 0~255의 값을 가지는데, 이를 Normalization (255로 나누어주면 됨)
* 결과값에 대해서는 One-hot-Encoding 적용

코드로는 다음과 같습니다.

<pre class="prettyprint">
(X_train, Y_train), (X_validation, Y_validation) = mnist.load_data()

X_train = X_train.reshape(X_train.shape[0], 784).astype('float64') / 255
X_validation = X_validation.reshape(X_validation.shape[0], 784).astype(
    'float64') / 255

Y_train = np_utils.to_categorical(Y_train, 10)
Y_validation = np_utils.to_categorical(Y_validation, 10)
</pre>

그 다음부터는 다른 딥러닝 코드들과 비슷한 흐름으로 흘러갑니다.

<br>

## 전체 소스

`ModelCheckpoint` 및 `EarlyStopping` 모듈까지 적용한 전체 소스입니다.

<pre class="prettyprint">
from keras.datasets import mnist
from keras.utils import np_utils
from keras.models import Sequential
from keras.layers import Dense
from keras.callbacks import ModelCheckpoint, EarlyStopping

import os

MODEL_SAVE_FOLDER_PATH = './model/'

if not os.path.exists(MODEL_SAVE_FOLDER_PATH):
  os.mkdir(MODEL_SAVE_FOLDER_PATH)

model_path = MODEL_SAVE_FOLDER_PATH + 'mnist-' + '{epoch:02d}-{val_loss:.4f}.hdf5'

cb_checkpoint = ModelCheckpoint(filepath=model_path, monitor='val_loss',
                                verbose=1, save_best_only=True)

cb_early_stopping = EarlyStopping(monitor='val_loss', patience=10)

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

model.fit(X_train, Y_train, validation_data=(X_validation, Y_validation),
          epochs=30, batch_size=200, verbose=0,
          callbacks=[cb_checkpoint, cb_early_stopping])

print('\nAccuracy: {:.4f}'.format(model.evaluate(X_validation, Y_validation)[1]))
</pre>