---
layout: post
title: 최적 모델 찾기(Karas)
category: 머신러닝
permalink: /machine-learning/:year/:month/:day/:title/

tag: [머신러닝, Python, Keras]
---
# 와인 감별 머신 러닝

와인의 속성을 체크하여 레드 와인과 화이트 와인을 구분하는 예제입니다. 학습에 사용하는 데이터 세트는 [여기](/assets/machine-learning/wine.csv)에서 받을 수 있습니다.

<br>

## 기본 예제 코드

먼저 아주 기본적인 머신러닝 코드를 구현합니다.

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense

import pandas as pd

df = pd.read_csv('wine.csv', header=None)
df = df.sample(frac=1)
data_set = df.values

X = data_set[:, 0:12]
Y = data_set[:, 12]

model = Sequential()
model.add(Dense(30, input_dim=12, activation='relu'))
model.add(Dense(12, activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy', optimizer='adam',
              metrics=['accuracy'])

model.fit(X, Y, epochs=200, batch_size=200)

print('\nAccuracy: {:.4f}'.format(model.evaluate(X, Y)[1]))
</pre>

<br>

## 모델의 저장과 불러오기

모델은 다음 코드를 이용해서 저장하고 불러올 수 있습니다.

<pre class="prettyprint">
from kera.models import load_model

# 모델 저장
model.save('snowdeer_model.h5')

# 모델 불러오기
model = load_model('snowdeer_model.h5')
</pre>

<br>

## 모델 체크포인터 콜백

`ModelCheckpoint` 콜백 함수는 Keras에서 모델을 학습할 때마다 중간중간에 콜백 형태로 알려줍니다. 다음과 같은 코드를 이용해서 사용할 수 있습니다.

<pre class="prettyprint">
from keras.callbacks import ModelCheckpoint
import os

# ...

MODEL_SAVE_FOLDER_PATH = './model/'
if not os.path.exists(MODEL_SAVE_FOLDER_PATH):
  os.mkdir(MODEL_SAVE_FOLDER_PATH)

model_path = MODEL_SAVE_FOLDER_PATH + '{epoch:02d}-{val_loss:.4f}.hdf5'

cb_checkpoint = ModelCheckpoint(filepath=model_path, monitor='val_loss',
                                verbose=1, save_best_only=True)

# ...

model.fit(X, Y, validation_split=0.2, epochs=200, batch_size=200, verbose=0,
          callbacks=[cb_checkpoint])
</pre>

`ModelCheckpoint`의 속성으로 `verbose`는 해당 함수의 진행 사항의 출력 여부, `save_best_only`는 모델의 정확도가 최고값을 갱신했을 때만 저장하도록 하는 옵션입니다.

전체 코드는 다음과 같습니다.

<pre class="prettyprint">
from keras.models import Sequential
from keras.layers import Dense
from keras.callbacks import ModelCheckpoint
import os
import pandas as pd

MODEL_SAVE_FOLDER_PATH = './model/'

df = pd.read_csv('wine.csv', header=None)
df = df.sample(frac=1)
data_set = df.values

X = data_set[:, 0:12]
Y = data_set[:, 12]

if not os.path.exists(MODEL_SAVE_FOLDER_PATH):
  os.mkdir(MODEL_SAVE_FOLDER_PATH)

model_path = MODEL_SAVE_FOLDER_PATH + '{epoch:02d}-{val_loss:.4f}.hdf5'

cb_checkpoint = ModelCheckpoint(filepath=model_path, monitor='val_loss',
                                verbose=1, save_best_only=True)

model = Sequential()
model.add(Dense(30, input_dim=12, activation='relu'))
model.add(Dense(12, activation='relu'))
model.add(Dense(8, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.compile(loss='binary_crossentropy', optimizer='adam',
              metrics=['accuracy'])

model.fit(X, Y, validation_split=0.2, epochs=200, batch_size=200, verbose=0,
          callbacks=[cb_checkpoint])

print('\nAccuracy: {:.4f}'.format(model.evaluate(X, Y)[1]))
</pre>

실행해보면 `./model` 폴더 아래에 수많은 `.hdf5` 파일들이 생겼음을 확인할 수 있습니다. 파일명은 `epoch` 값과 `val_loss(오차율)`이므로 가장 최종적으로 생긴 파일이 가장 성능이 좋은 모델이 됩니다.

<br>

## 학습 자동 중단

반복문을 돌면서 최고 성능의 모델을 찾아낼 때, 초반에 최고 성능의 모델이 찾아져서 그 보다 더 좋은 성능의 모델이 더 이상 발견되지 않는 경우가 있습니다. 이런 경우 학습을 중단하도록 하는 기능을 Keras에서는 `EarlyStopping()`이라는 함수로 제공하고 있습니다. 

<pre class="prettyprint">
from keras.callbacks import EarlyStopping

# ...

cb_early_stopping = EarlyStopping(monitor='val_loss', patience=100)

# ...

model.fit(X, Y, validation_split=0.2, epochs=5000, batch_size=500,
          callbacks=[EarlyStopping])
</pre>

위와 같은 코드를 이용해서 구현하면 `epoch` 횟수가 총 5,000번이지만, 중간에 더 좋은 성능의 모델이 100번동안 발견이 되지 않으면 학습을 멈추게 됩니다.